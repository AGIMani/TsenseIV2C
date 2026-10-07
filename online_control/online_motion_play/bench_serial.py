"""Loopback transport benchmark using a private server and simulated H7 only.

Compare legacy 20 ms JSON exchange against event-driven binary exchange with
the same 500 Hz A3 planner. No physical port, production service or saved data.
"""
import argparse
import base64
import http.client
import json
from pathlib import Path
import socket
import statistics
import struct
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from a30_kinematics.downlink import pack_aa55, crc16_modbus


def benchmark(port, mode, seconds):
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
    headers = {"X-Motion-Client":"benchmark-serial-client", "Content-Type":"application/json"}
    def api(action, body=None, raw=False):
        connection.request("GET" if body is None else "POST", "/api/"+action,
                           body if raw else None if body is None else json.dumps(body),
                           {**headers, **({"Content-Type":"application/octet-stream"} if raw else {})})
        response = connection.getresponse()
        data = response.read()
        if response.status != 200:
            raise RuntimeError(data.decode())
        return data if raw else json.loads(data)
    token, seq = None, 0
    def exchange():
        nonlocal seq
        seq += 1
        payload = bytearray(484)
        struct.pack_into("<HHII", payload, 0, seq & 65535, 0, seq, 0)
        telemetry = pack_aa55(2, payload)
        if mode == "binary":
            response = api("serial_binary", struct.pack("<4s32sI", b"MSR1", token.encode(), seq)+telemetry, raw=True)
            magic, echoed_token, echoed_seq, count = struct.unpack_from("<4s32sIH", response)
            assert magic == b"MST1" and echoed_token.decode() == token and echoed_seq == seq
            offset, frames = 42, []
            for _ in range(count):
                length, age = struct.unpack_from("<Hf", response, offset)
                frames.append((response[offset+6:offset+6+length], age))
                offset += 6+length
            assert offset == len(response)
        else:
            response = api("serial", {"token":token, "seq":seq, "rx":base64.b64encode(telemetry).decode()})["result"]
            frames = [(base64.b64decode(item["data"]), item["age_ms"]) for item in response["frames"]]
        for frame, age in frames:
            assert crc16_modbus(frame[:-2]) == struct.unpack_from("<H", frame, len(frame)-2)[0]
        return [(frame, age) for frame, age in frames if frame[5] == 0x20]
    try:
        api("control", {})
        api("model", {"model_id":"TsenseI_v3_0"})
        token = api("connect", {"port":"browser"})["result"]["token"]
        exchange()
        state = api("state")
        api("widget", {"id":state["controls"]["realtimeJointControl"]["id"], "value":True})
        exchange()  # Exclude startup/enable frames.
        produced_before = api("state")["device"]["tx"]
        started, requests, positions = time.perf_counter(), 0, 0
        latency, age_at_receipt = [], []
        while time.perf_counter()-started < seconds:
            at = time.perf_counter()
            frames = exchange()
            elapsed = time.perf_counter()-at
            latency.append(elapsed*1000)
            age_at_receipt.extend(age+elapsed*1000 for _, age in frames)
            positions += len(frames)
            requests += 1
            if mode == "legacy_json_20ms":
                time.sleep(max(0., .02-elapsed))
        duration = time.perf_counter()-started
        state = api("state")
        assert state["device"]["armed"], state["device"]["error"]
        percentile = lambda values: sorted(values)[min(len(values)-1, int(len(values)*.95))]
        return {"mode":mode, "duration_s":round(duration,3), "position_hz":round(positions/duration,1),
                "server_produced_hz":round((state["device"]["tx"]-produced_before)/duration,1),
                "exchange_hz":round(requests/duration,1), "rtt_mean_ms":round(statistics.mean(latency),3),
                "rtt_p95_ms":round(percentile(latency),3),
                "command_age_p95_upper_bound_ms":round(percentile(age_at_receipt),3),
                "telemetry_upload_kib_s":round(requests*492/duration/1024,1)}
    finally:
        if token:
            api("disconnect", {"token":token})
        api("release", {})
        connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=3.)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--browser", action="store_true", help="Also benchmark the JS client with Node and mock Web Serial")
    args = parser.parse_args()
    if not 1 <= args.seconds <= 30:
        parser.error("seconds must be in [1,30]")
    with tempfile.TemporaryDirectory(prefix="motion-serial-bench-") as directory:
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            port = listener.getsockname()[1]
        config = Path(directory)/"config.json"
        config.write_text(json.dumps({"host":"127.0.0.1", "port":port, "data_dir":directory}))
        process = subprocess.Popen([sys.executable, str(Path(__file__).with_name("server.py")),
                                    "--config", str(config), "--no-browser"],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            for _ in range(100):
                try:
                    with socket.create_connection(("127.0.0.1", port), timeout=.1):
                        break
                except OSError:
                    if process.poll() is not None:
                        raise RuntimeError("Benchmark server exited")
                    time.sleep(.1)
            result = {"scope":"Loopback HTTP + Python/Ruckig + simulated H7, no physical USB or browser rendering",
                      "planner_hz":500, "cases":[benchmark(port, mode, args.seconds) for mode in ("legacy_json_20ms", "binary")]}
            if args.browser:
                output = subprocess.check_output(["node", "--experimental-default-type=module",
                    str(Path(__file__).with_name("bench_browser_serial.mjs")), f"http://127.0.0.1:{port}", str(args.seconds)],
                    text=True, timeout=args.seconds+15)
                result["js_client"] = json.loads(output)
                result["latency_cases"] = []
                for mode in ("binary", "stream"):
                    output = subprocess.check_output(["node", "--experimental-default-type=module",
                        str(Path(__file__).with_name("bench_browser_serial.mjs")), f"http://127.0.0.1:{port}",
                        str(args.seconds), mode, "10"], text=True, timeout=args.seconds+15)
                    result["latency_cases"].append(json.loads(output))
            text = json.dumps(result, indent=2)
            print(text)
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(text+"\n", encoding="utf-8")
        finally:
            process.terminate()
            process.wait(timeout=10)


if __name__ == "__main__":
    main()
