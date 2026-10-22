"""A3 feedback-based startup, Ruckig limits, collision and host isolation."""
from pathlib import Path
import json
import math
import struct
import sys
import time
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/"src")]
from a30_kinematics import A30StrokeMapper
from a30_kinematics.downlink import channel_mapping
from a30_kinematics.feedback import measured_pose
from hand_profiles import load_hand_profile
from online_control.online_motion_track.a30.control import A30Control
from online_control.online_motion_track.main import Application


@pytest.mark.parametrize("side", ["left", "right"])
def test_gesture_limits_for_every_axis(side):
    c = A30Control(load_hand_profile("TsenseI_v3_0"), side, "off", start=False)
    try:
        counts = {"flexion": 0, "swing": 0, "rotation": 0}
        for name, v, a, j in zip(c.names, c.input.max_velocity, c.input.max_acceleration, c.input.max_jerk):
            if name.endswith(("_MCP_joint", "_PIP_joint", "_DIP_joint")):
                kind = "flexion"
            elif name.endswith("_swing_joint"):
                kind = "swing"
            else:
                assert "thumb_rota" in name, name
                kind = "rotation"
            counts[kind] += 1
            assert (v, a, j) == (20., 400., 2400.), name
        assert counts == {"flexion": 14, "swing": 4, "rotation": 2}
        import ruckig
        assert c.input.synchronization == ruckig.Synchronization.Time
        assert c.otg.delta_time == .002
    finally:
        c.close()


@pytest.mark.parametrize("side", ["left", "right"])
def test_gesture_trajectory_respects_derivative_limits(side):
    import numpy as np
    c = A30Control(load_hand_profile("TsenseI_v3_0"), side, "off", start=False)
    try:
        previous_acceleration = np.zeros(20)
        for tick in range(450):
            fraction = .8 if tick < 225 else .1
            target = {name: lo + fraction * (hi - lo) for name, (lo, hi) in c.limits.items()}
            c.push(target, tick)
            c.step(.002)
            if not c.valid:
                assert c.phase == 'confirming'
                continue
            velocity = np.asarray(c.output.new_velocity)
            acceleration = np.asarray(c.output.new_acceleration)
            assert np.all(np.abs(velocity) <= np.asarray(c.input.max_velocity) + 1e-8)
            assert np.all(np.abs(acceleration) <= np.asarray(c.input.max_acceleration) + 1e-8)
            assert np.all(np.abs(acceleration - previous_acceleration) / .002 <= np.asarray(c.input.max_jerk) + 1e-6)
            previous_acceleration = acceleration.copy()
    finally:
        c.close()


def feedback(mapper, pose):
    t=mapper.convert(pose); b=bytearray(484)
    struct.pack_into("<HHII",b,0,7,3,1000,0x3ffff)
    for r in channel_mapping(mapper.hand_side):
        v=(t.linear_strokes_mm if r["unit"]=="mm" else t.rotary_positions_deg)[r["joint"]]
        if r["field"]=="self_pos":
            v *= r["position_sign"]
            struct.pack_into("<4B4f2h",b,12+r["slot"]*24,0,0,0,2,v,0,0,v,0,2500)
        else:
            o=444+r["slot"]*20;b[o:o+2]=bytes([5+4*r["slot"],5])
            struct.pack_into("<IHH",b,o+2,round(v*r["gear_ratio"]/360*2097152)+0x80000000,0x8000,0x8000)
            b[o+16]=4
    return bytes(b)


@pytest.mark.parametrize("model_id", ["TsenseI_v3_0", "TsenseI_v3_0_withid"])
@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("collision_mode", ["stop", "mink"])
def test_protected_a30_can_rebase_before_first_collision_frame(model_id, side, collision_mode):
    profile = load_hand_profile(model_id)
    c = A30Control(profile, side, collision_mode, start=False)
    try:
        seed = profile.init_pose(side)
        assert c.guard.guard._previous_safe == seed
        joint = f"{side}_hand_index_PIP_joint"
        seed[joint] = .1
        # Feedback arrives before any control sample has opened a cache frame.
        c.rebase(feedback(c.mapper, seed))
        assert c.pose[joint] == pytest.approx(.1, abs=1e-5)
        assert c.guard.guard._previous_safe == c.pose
        c.push({**c.pose, joint: .2}, 1)
        c.step(.002)
        assert c.valid and c.ticks == 1 and len(c.wire) == 108
        assert c.pose[joint] > .1
        assert c.sample_collision["output_max_penetration_mm"] == 0
        c.rebase(feedback(c.mapper, seed))
        assert c.guard.guard._previous_safe == c.pose
    finally:
        c.close()


@pytest.mark.parametrize("model_id", ["TsenseI_v3_0", "TsenseI_v3_0_withid"])
@pytest.mark.parametrize("collision_mode", ["off", "monitor"])
@pytest.mark.parametrize("direct_mode", [False, True])
def test_unprotected_a30_publishes_bounded_wire_and_sampling_fault_still_latches(model_id, collision_mode, direct_mode):
    c = A30Control(load_hand_profile(model_id), "left", collision_mode, start=False)
    try:
        assert c.guard is None
        c.set_direct_mode(direct_mode)
        c.rebase(feedback(c.mapper, c.pose))
        target = {**c.pose, "left_hand_index_PIP_joint": .5}
        c.push(target, 1)
        c.step(.002)
        assert c.valid and len(c.wire) == 108
        assert c.pose["left_hand_index_PIP_joint"] > 0
        assert all(lo <= c.pose[n] <= hi for n, (lo, hi) in c.limits.items())
        state = c.snapshot()
        assert not state["sample_collision_ready"]
        assert state["sample_collision"]["checked"] is False
        assert state["sample_collision"]["protection_enabled"] is False
        c.step(.2)
        assert not c.valid and not c.wire and "超时" in c.error
        c.push(target, 2)
        c.step(.002)
        assert not c.valid and not c.wire
    finally:
        c.close()


def test_stream_recovery_uses_last_command_instead_of_cached_feedback():
    c = A30Control(load_hand_profile('TsenseI_v3_0'), 'left', 'stop', start=False)
    try:
        initial = dict(c.pose)
        commanded = dict(initial); commanded['left_hand_mid_PIP_joint'] = .12
        payload = feedback(c.mapper, initial)
        state = c.rebase(payload, commanded, resume_from_commanded_pose=True)
        assert state['ruckig_joint_angles_rad'] == pytest.approx(commanded, abs=1e-12)
        # Manual enabling still starts from actual available telemetry.
        state = c.rebase(payload, commanded)
        assert state['ruckig_joint_angles_rad']['left_hand_mid_PIP_joint'] == pytest.approx(0, abs=1e-5)
        for invalid in (None, {}, {**commanded, 'left_hand_mid_PIP_joint': float('nan')},
                        {**commanded, 'left_hand_mid_PIP_joint': 99}):
            with pytest.raises(ValueError):
                c.rebase(payload, invalid, resume_from_commanded_pose=True)
    finally:
        c.close()


@pytest.mark.parametrize("side",["left","right"])
def test_actual_feedback_inverse_roundtrip_all_channels(side):
    m=A30StrokeMapper(side)
    for frac in (0,.2,.7,1):
        pose={n:lo+(hi-lo)*frac for n,(lo,hi) in zip(m.joint_names,m._limits)}
        actual=measured_pose(feedback(m,pose),m)
        assert max(abs(actual[n]-pose[n]) for n in pose)<1e-5


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("upper", [False, True])
def test_feedback_clamps_every_actuator_to_model_endpoints(side, upper):
    m = A30StrokeMapper(side)
    initial = load_hand_profile("TsenseI_v3_0").init_pose(side)
    b = bytearray(feedback(m, initial))
    rows = channel_mapping(side)
    for row in rows:
        if row["field"] == "self_pos":
            struct.pack_into("<f", b, 16 + row["slot"]*24, (10000. if upper else -10000.) * row["position_sign"])
        else:
            struct.pack_into("<I", b, 446 + row["slot"]*20, 0xffffffff if upper else 0)
    clipped = []
    pose = measured_pose(b, m, clamp=True, clipped_joints=clipped)
    assert len(clipped) == 20
    converted = m.convert(pose)
    for item in clipped:
        targets = converted.linear_strokes_mm if item["unit"] == "mm" else converted.rotary_positions_deg
        assert item["applied"] == (item["upper"] if upper else item["lower"])
        assert targets[item["joint"]] == pytest.approx(item["applied"], abs=.002)
    assert all(lo-1e-8 <= pose[n] <= hi+1e-8 for n, (lo,hi) in zip(m.joint_names,m._limits))


def test_web_rebase_negative_pinky_feedback_clamps_to_zero_and_preserves_diagnostics():
    c = A30Control(load_hand_profile("TsenseI_v3_0"), "left", "stop", start=False)
    try:
        b = bytearray(feedback(c.mapper, c.profile.init_pose()))
        struct.pack_into("<f", b, 16, -.16041614)
        state = c.rebase(b)
        name = "left_hand_pinky_DIP_joint"
        assert state["ruckig_joint_angles_rad"][name] == 0
        assert state["rotary_positions_deg"][name] == 0
        detail = next(x for x in state["feedback_clipped_joints"] if x["joint"] == name)
        assert detail["raw"] == pytest.approx(-.16041614)
        assert detail["applied"] == 0 and detail["unit"] == "deg"
        assert state["feedback_position_limits"]["rotary"][name] == pytest.approx([0,math.degrees(1.57)])
    finally:
        c.close()


@pytest.mark.parametrize("mutation",["missing","fault","nan","length","range"])
def test_feedback_refuses_unusable_seed(mutation):
    m=A30StrokeMapper(); b=bytearray(feedback(m,load_hand_profile("TsenseI_v3_0").init_pose()))
    if mutation=="missing":struct.pack_into("<I",b,8,0)
    if mutation=="fault":struct.pack_into("<I",b,456,4)
    if mutation=="nan":struct.pack_into("<f",b,16,float("nan"))
    if mutation=="length":b=b[:-1]
    if mutation=="range":struct.pack_into("<f",b,16,1000)
    with pytest.raises(ValueError):measured_pose(b,m)


@pytest.mark.parametrize("side", ["left", "right"])
def test_partial_feedback_uses_fallback_only_for_missing_axes(side):
    m = A30StrokeMapper(side)
    base = load_hand_profile("TsenseI_v3_0").init_pose(side)
    actual = dict(base); actual[f"{side}_hand_mid_DIP_joint"] = .2
    b = bytearray(feedback(m, actual))
    struct.pack_into("<I", b, 8, 0x3fffe)
    struct.pack_into("<H", b, 2, 0)
    struct.pack_into("<f", b, 16, float("nan"))
    b[444:] = bytes(40)
    missing = []
    pose = measured_pose(b, m, fallback_pose=base, missing_joints=missing)
    assert len(missing) == 3
    assert pose[f"{side}_hand_mid_DIP_joint"] == pytest.approx(.2, abs=1e-5)
    assert all(pose[n] == base[n] for n in missing)


@pytest.mark.parametrize("side", ["left", "right"])
def test_self_only_444_byte_firmware_rebase(side):
    c=A30Control(load_hand_profile("TsenseI_v3_0"),side,"stop",start=False)
    try:
        initial=c.profile.init_pose(side)
        payload=feedback(c.mapper,initial)[:444]
        result=c.rebase(payload.hex())
        assert len(result["feedback_fallback_joints"])==2
        assert all("thumb_rota" in n for n in result["feedback_fallback_joints"])
        assert result["ruckig_joint_angles_rad"]==pytest.approx(initial,abs=1e-5)
    finally:c.close()


def test_web_rebase_all_missing_uses_initial_or_last_sent_pose_not_preview():
    c = A30Control(load_hand_profile("TsenseI_v3_0"), "left", "stop", start=False)
    try:
        b = bytes(484)
        initial = c.profile.init_pose()
        c.pose["left_hand_mid_DIP_joint"] = .4  # Unsent preview is not feedback.
        state = c.rebase(b)
        assert state["ruckig_joint_angles_rad"] == initial
        assert len(state["feedback_fallback_joints"]) == 20
        held = dict(initial); held["left_hand_mid_DIP_joint"] = .1
        assert c.rebase(b, held)["ruckig_joint_angles_rad"] == held
        broken = dict(held); broken["left_hand_mid_DIP_joint"] = float("nan")
        with pytest.raises(ValueError): c.rebase(b, broken)
        with pytest.raises(ValueError): c.rebase(b, [])
        fault = bytearray(feedback(c.mapper, initial)); struct.pack_into("<I", fault, 456, 4)
        with pytest.raises(ValueError, match="故障"): c.rebase(fault)
    finally: c.close()


def test_online_external_status_matches_source_mask_or_identifier_rule():
    m = A30StrokeMapper()
    initial = load_hand_profile("TsenseI_v3_0").init_pose()
    b = bytearray(feedback(m, initial))
    # A3_USB_Downlink: valid = mit_ident_ok(raw, idx) or alive.
    b[444:446] = b"\x00\x00"
    assert measured_pose(b, m, fallback_pose=initial) == pytest.approx(initial, abs=1e-5)
    struct.pack_into("<I", b, 456, 4)
    with pytest.raises(ValueError, match="故障"):
        measured_pose(b, m, fallback_pose=initial)


def test_ruckig_is_bounded_and_rebases_to_feedback_not_target():
    c=A30Control(load_hand_profile("TsenseI_v3_0"),"left","stop",start=False)
    try:
        seed=c.profile.init_pose();c.rebase(feedback(c.mapper,seed).hex())
        target=dict(seed);target["left_hand_mid_PIP_joint"]=.25
        for i in range(3):c.push(target,i)
        previous=c.pose.copy();previous_v=[0.]*20;previous_a=[0.]*20
        for i in range(50):
            c.target_at=time.perf_counter();c.step(.002)
            assert c.valid,c.error
            assert all(abs(c.pose[n]-previous[n])<=v*.002+1e-8 for n,v in zip(c.names,c.input.max_velocity))
            velocity=c.input.current_velocity;accel=c.input.current_acceleration
            assert all(abs(v-pv)<=a*.002+1e-8 for v,pv,a in zip(velocity,previous_v,c.input.max_acceleration))
            assert all(abs(a-pa)<=j*.002+1e-8 for a,pa,j in zip(accel,previous_a,c.input.max_jerk))
            previous=c.pose.copy();previous_v=velocity;previous_a=accel
        assert 0<c.pose["left_hand_mid_PIP_joint"]<.25
        actual=dict(seed);actual["left_hand_mid_PIP_joint"]=.1
        state=c.rebase(feedback(c.mapper,actual).hex())
        assert state["browser_epoch"]==2 and state["target_ready"]
        assert state["tracking_transition"]["phase"] == "holding"
        assert state["ruckig_joint_angles_rad"]["left_hand_mid_PIP_joint"]==pytest.approx(.1,abs=1e-5)
        for i in range(3):c.push(target,i)
        c.step(.2)
        assert not c.valid and c.phase=="blocked"
    finally:c.close()


def test_post_ruckig_collision_never_commits_unsafe_sample():
    c=A30Control(load_hand_profile("TsenseI_v3_0"),"left","stop",start=False)
    try:
        old=c.pose.copy();target=dict(old);target["left_hand_mid_PIP_joint"]=.2
        for i in range(3):c.push(target,i)
        c.guard.project=lambda pose:dict(old)
        c.step(.02)
        assert c.pose==old and c.valid and c.phase=="collision_limited"
        assert len(c.wire) == 108
        c.hold_target()
        assert c.snapshot()["target_ready"] and c.phase == "collision_limited"
    finally:c.close()


@pytest.mark.parametrize("mode", ["stop", "mink"])
def test_collision_pause_keeps_stream_and_resumes_only_after_checked_sample(mode):
    c = A30Control(load_hand_profile("TsenseI_v3_0"), "left", mode, start=False)
    stream = c.open_stream(c.epoch)
    try:
        c.step(.002)
        first = next(stream)
        assert len(first) == 216 and struct.unpack_from('<I', first, 24)[0] == 1
        old, ticks = dict(c.pose), c.ticks
        target = dict(old); target["left_hand_mid_PIP_joint"] = .2
        c.guard.project = lambda pose: dict(old)
        c.push(target, 1); c.step(.02)
        assert c.pose == old and c.ticks == ticks+1 and len(c.wire) == 108
        assert c.input.current_velocity == [0.]*20
        assert c.input.current_acceleration == [0.]*20
        for _ in range(2):
            c.hold_target(); c.step(.002)  # Missing input cannot unpause.
            paused = next(stream)
            assert len(paused) == 216 and struct.unpack_from('<I', paused, 24)[0] == 1
        c.push(target, 2)
        # An unchanged gesture keeps the fingers held; safe holds still stream.
        assert c.snapshot()["target_ready"]
        assert struct.unpack_from('<I', next(stream), 24)[0] == 1
        c.step(.02)
        assert c.phase == 'collision_limited' and c.pose == old
        assert c.stream_active
        # The next valid observation can recover within the same epoch/stream.
        c.guard.project = lambda pose: dict(pose)
        target = {n: .3 if n.endswith('mid_PIP_joint') else q+.01 for n, q in target.items()}
        c.push(target, 3)
        c.step(.002)
        resumed = next(stream)
        assert len(resumed) == 216 and struct.unpack_from('<I', resumed, 24)[0] == 1
        assert c.ticks > ticks+1 and c.phase == 'tracking' and not c.error
        assert c.snapshot()["target_ready"]
        assert 0 < c.pose["left_hand_mid_PIP_joint"]-old["left_hand_mid_PIP_joint"] < .001
        assert all(abs(v) <= limit for v, limit in zip(c.input.current_velocity, c.input.max_velocity))
        assert all(abs(a) <= jerk*.002+1e-8 for a, jerk in zip(c.input.current_acceleration, c.input.max_jerk))
    finally:
        stream.close(); c.close()


def test_hard_fault_cannot_be_cleared_by_new_hand_or_hold():
    c = A30Control(load_hand_profile("TsenseI_v3_0"), "left", "stop", start=False)
    try:
        c.step(.002)
        c.step(.2)
        old, ticks = dict(c.pose), c.ticks
        c.push(old, 1); c.hold_target(); c.step(.002)
        assert not c.valid and c.phase == 'blocked' and not c.wire
        assert c.snapshot()['rebase_required']
        assert c.error == '控制采样超时' and c.ticks == ticks
        stream = c.open_stream(c.epoch)
        record = next(stream); stream.close()
        assert len(record) == 108 and struct.unpack_from('<I', record, 24)[0] == 2
        c.rebase(feedback(c.mapper, old)); c.step(.002)
        assert c.valid and c.wire
        assert not c.snapshot()['rebase_required']
    finally:
        c.close()


@pytest.mark.parametrize("mode", ["stop", "mink"])
def test_convex_thumb_pinky_collision_recovers_with_real_ruckig_and_geometry(mode):
    # Retain the original frame-685 recording as historical evidence. Its
    # capsule-safe seed overlaps the fitted meshes, so this derived fixture
    # retracts the thumb before replaying the same separating-finger scenario.
    case = json.loads((ROOT / 'tests/fixtures/a30_convex_thumb_pinky.json').read_text(encoding='utf-8'))
    c = A30Control(load_hand_profile("TsenseI_v3_0"), "left", mode, start=False)
    try:
        c.guard.filter.sync_pose_from_dict(case['target'])
        assert any({p['g1'], p['g2']} == set(case['expected_contact']) and p['pen_mm'] > 0
                   for p in c.guard.filter.contact_summary())
        c._seed(case['last_safe_pose'])
        assert c.guard.filter.data.ncon == 0
        c.push(case['target'], 685)
        for _ in range(200):
            c.step(.002)
            if c.phase == 'collision_limited':
                break
        assert c.phase == 'collision_limited' and len(c.wire) == 108
        # The pinky can separate safely while the thumb stays stopped. A
        # contact pair must not automatically freeze both kinematic chains.
        assert set(c.finger_holds) == {'thumb'}
        c.guard.filter.sync_pose_from_dict(c.pose)
        assert c.guard.filter.data.ncon == 0
        contacts = c.sample_collision['contacts']
        assert any({p['g1'], p['g2']} == set(case['expected_contact']) for p in contacts)
        frozen = dict(c.pose)
        for _ in range(80):
            c.step(.002)
            assert c.valid and set(c.finger_holds) == {'thumb'}, c.error
            assert all(c.pose[n] == frozen[n] for n in c.names if 'hand_thumb_' in n)
            c.guard.filter.sync_pose_from_dict(c.pose)
            assert c.guard.filter.data.ncon == 0
        assert c.pose['left_hand_pinky_DIP_joint'] > frozen['left_hand_pinky_DIP_joint'] + .05
        c.push(case['recovery'], 686)
        for _ in range(500):
            c.step(.002)
            assert c.valid and c.phase != 'collision_limited', c.error
            assert c.guard.diagnostics['output_max_penetration_mm'] == 0
        assert c.ticks > 500 and c.pose != frozen and len(c.wire) == 108
        assert c.pose == pytest.approx(case['recovery'], abs=1e-8)
    finally:
        c.close()


def test_hold_without_hand_and_first_frame_resume_keep_ruckig_continuous():
    c=A30Control(load_hand_profile("TsenseI_v3_0"),"left","stop",start=False)
    try:
        initial=c.profile.init_pose()
        state=c.rebase(feedback(c.mapper,initial))
        assert state["target_ready"] and c.phase == "holding"
        c.step(.002)
        with_stream=c.open_stream(c.epoch)
        record=next(with_stream);with_stream.close()
        assert struct.unpack_from('<I',record,24)[0] & 1
        assert len(record)==216 and c.pose == pytest.approx(initial,abs=1e-5)
        target=dict(initial);target["left_hand_mid_DIP_joint"]=.2
        c.push(target,1)  # First valid observation immediately becomes the target.
        assert c.valid and c.phase=="tracking"
        for _ in range(40):c.step(.002)
        velocity=list(c.input.current_velocity);acceleration=list(c.input.current_acceleration)
        c.target_at=time.perf_counter()-10
        c.hold_target()
        assert c.input.current_velocity==velocity and c.input.current_acceleration==acceleration
        for _ in range(600):c.step(.002)
        assert c.valid and c.phase=="holding"
        assert c.pose==pytest.approx(target,abs=1e-5)
        assert c.snapshot()["target_ready"]
        stream=c.open_stream(c.epoch);record=next(stream);stream.close()
        assert struct.unpack_from('<I',record,24)[0] & 1 and len(record)==216
        c.push(initial,2);c.step(.002)
        assert c.valid and c.phase=="tracking"
    finally:c.close()


@pytest.mark.parametrize("mode", ["stop", "mink"])
def test_public_a30_api_never_opens_server_usb(mode):
    app=Application(public=True)
    try:
        s=app.create({"model_id":"TsenseI_v3_0","recording":False,"collision_mode":mode})
        obj=app.get(s["session"])
        with obj.control.lock:
            assert obj.control.preview_only
            obj.control.step(.3)
            assert obj.control.valid and not obj.control.error
        request=lambda action,**kw:app.a30_command({"session":s["session"],"action":action,**kw})
        assert request("status")["ruckig_enabled"]
        request("tracking_start")
        with pytest.raises(ValueError):request("browser_rebase",pose=s["init_pose"])
        result=request("browser_rebase",telemetry=feedback(obj.control.mapper,s["init_pose"]).hex())
        assert result["browser_epoch"]==1
        for seq,label in enumerate((None,"Right")):
            hand=None if label is None else {"label":label,"world":[[0,0,0]]*21,"normalized":[[0,0,0]]*21}
            result=obj.process({"sequence":seq,"timestamp_ms":1000+seq*1000,
                                "capture_unix_ms":time.time()*1000,"hand":hand})
            assert not result["tracking"] and result["hardware"]["target_ready"]
            assert result["hardware"]["tracking_transition"]["phase"]=="holding"
        if mode == "mink":
            guard = obj.pipeline.mink_guard
            assert max(abs(guard.q-guard._vector(result["hardware"]["ruckig_joint_angles_rad"]))) < 1e-8
        partial=request("browser_rebase",telemetry=bytes(484).hex(),pose=s["init_pose"])
        assert len(partial["feedback_fallback_joints"]) == 20
        for action in ("ports","connect","home","arm","disconnect"):
            with pytest.raises(ValueError):request(action)
        app.remove(s["session"])
        assert not obj.control.thread.is_alive()
    finally:app.close()


def test_lite_bundle_never_advertises_hardware_with_only_installed_libraries(monkeypatch):
    from online_control.online_motion_track import main
    monkeypatch.setattr(main, "a30_collision_modes", lambda: ["off"])
    assert not main.a30_control_available()


@pytest.mark.parametrize("model_id", ["TsenseI_v3_0", "TsenseI_v3_0_withid"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_offline_preview_resumes_after_scheduler_gap_without_catching_up(model_id, side):
    profile = load_hand_profile(model_id)
    preview = A30Control(profile, side, "off", start=False, preview_only=True)
    reference = A30Control(profile, side, "off", start=False, preview_only=True)
    try:
        target = {**preview.pose, f"{side}_hand_index_PIP_joint": .5}
        for c in (preview, reference):
            c.push(target, 1)
            c.step(.002)
        preview.step(.3)
        reference.step(.002)
        assert preview.valid and not preview.error
        assert preview.pose == pytest.approx(reference.pose)
        assert preview.input.current_velocity == pytest.approx(reference.input.current_velocity)
        assert preview.ticks == reference.ticks == 2
        for _ in range(200):
            preview.push(target, 2)
            preview.step(.002)
        assert preview.pose[f"{side}_hand_index_PIP_joint"] == pytest.approx(.5, abs=.01)
        stream = preview.open_stream(preview.epoch, preview=True)
        try:
            record = next(stream)
            assert len(record) == 108  # Display-only stream contains no USB payload.
            assert struct.unpack_from('<I', record, 24)[0] == 1
        finally:
            stream.close()
    finally:
        preview.close()
        reference.close()


@pytest.mark.parametrize("handoff", ["feedback", "stream"])
def test_preview_handoff_permanently_restores_hardware_timeout_protection(handoff):
    c = A30Control(load_hand_profile("TsenseI_v3_0"), "left", "off",
                   start=False, preview_only=True)
    try:
        if handoff == "feedback":
            c.rebase(feedback(c.mapper, c.pose))
        else:
            stream = c.open_stream(c.epoch)
            stream.close()
        assert not c.preview_only
        c.set_direct_mode(True)
        c.set_direct_mode(False)
        c.push({**c.pose, 'left_hand_index_PIP_joint': .5}, 1)
        c.step(.002)
        c.step(.3)
        c.push(c.pose, 2)
        c.step(.002)
        assert not c.valid and not c.wire
        assert c.error == "控制采样超时"
    finally:
        c.close()


@pytest.mark.parametrize("dt", [0, -.1, float("nan"), float("inf")])
def test_preview_still_latches_invalid_time(dt):
    c = A30Control(load_hand_profile("TsenseI_v3_0"), "left", "off",
                   start=False, preview_only=True)
    try:
        c.step(dt)
        c.push(c.pose, 1)
        c.step(.002)
        assert not c.valid and not c.wire and c.phase == "blocked"
    finally:
        c.close()
