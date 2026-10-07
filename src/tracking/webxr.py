"""Quest 3 WebXR 25-joint hand-skeleton input over same-origin HTTPS/WSS.

The Quest Browser performs hand tracking locally.  This module serves the
capture page and receives skeleton-only WebSocket messages on the same TLS
origin.  Raw WebXR 25-joint frames are preserved; conversion to the existing
MediaPipe-compatible 21-point control path is explicit and loss-aware.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from datetime import datetime, timedelta, timezone
import base64
from collections import deque
import hashlib
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import ssl
import struct
import threading
import time
from typing import Any, Callable, Mapping, Sequence
from urllib.parse import urlsplit

if __package__:
    from .cloudxr import CloudXRHandFrame
else:
    # The lightweight online server imports this parser directly from
    # src/tracking, without loading camera backends in tracking.__init__.
    from cloudxr import CloudXRHandFrame


WEBXR_PROTOCOL_SCHEMA = "teleop.webxr.hand_skeleton.v1"
WEBXR_JOINT_SET = "webxr-hand-25"
DEFAULT_WEBXR_HOST = "0.0.0.0"
DEFAULT_WEBXR_PORT = 8446
WEBXR_PATH = "/hand-skeleton"
WEBXR_STALE_TIMEOUT_S = 0.35
MAX_WEBSOCKET_MESSAGE_BYTES = 1_048_576
_WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
_PAGE_PATH = Path(__file__).with_name("webxr_capture.html")

_LOCAL_APPDATA = os.environ.get("LOCALAPPDATA")
if _LOCAL_APPDATA:
    DEFAULT_WEBXR_CERT_DIR = Path(_LOCAL_APPDATA) / "TsenseIHand" / "webxr"
else:
    DEFAULT_WEBXR_CERT_DIR = Path.home() / ".hand_dex" / "webxr"
DEFAULT_WEBXR_CERT = DEFAULT_WEBXR_CERT_DIR / "server.crt"
DEFAULT_WEBXR_KEY = DEFAULT_WEBXR_CERT_DIR / "server.key"
DEFAULT_WEBXR_CA_CERT = DEFAULT_WEBXR_CERT_DIR / "quest_webxr_ca.crt"
DEFAULT_WEBXR_CA_KEY = DEFAULT_WEBXR_CERT_DIR / "quest_webxr_ca.key"


WEBXR_HAND_JOINT_NAMES: tuple[str, ...] = (
    "wrist",
    "thumb-metacarpal",
    "thumb-phalanx-proximal",
    "thumb-phalanx-distal",
    "thumb-tip",
    "index-finger-metacarpal",
    "index-finger-phalanx-proximal",
    "index-finger-phalanx-intermediate",
    "index-finger-phalanx-distal",
    "index-finger-tip",
    "middle-finger-metacarpal",
    "middle-finger-phalanx-proximal",
    "middle-finger-phalanx-intermediate",
    "middle-finger-phalanx-distal",
    "middle-finger-tip",
    "ring-finger-metacarpal",
    "ring-finger-phalanx-proximal",
    "ring-finger-phalanx-intermediate",
    "ring-finger-phalanx-distal",
    "ring-finger-tip",
    "pinky-finger-metacarpal",
    "pinky-finger-phalanx-proximal",
    "pinky-finger-phalanx-intermediate",
    "pinky-finger-phalanx-distal",
    "pinky-finger-tip",
)

# WebXR Hand Input 25-joint order -> MediaPipe Hands 21-joint order.  The four
# non-thumb metacarpals are intentionally omitted; WebXR wrist remains index 0.
WEBXR_25_TO_MEDIAPIPE_21: tuple[int, ...] = (
    0,
    1, 2, 3, 4,
    6, 7, 8, 9,
    11, 12, 13, 14,
    16, 17, 18, 19,
    21, 22, 23, 24,
)


class WebXRProtocolError(ValueError):
    """Raised when a WebXR skeleton message violates the v1 schema."""


@dataclass(frozen=True)
class WebXRJoint:
    """One raw WebXR joint; invalid joints retain their name and validity."""

    name: str
    position: tuple[float, float, float] | None
    orientation: tuple[float, float, float, float] | None
    radius: float | None
    valid: bool
    derived: bool = False


@dataclass(frozen=True)
class WebXRCalibrationTransform:
    """Optional derived rigid transform for a fixed Quest/headset placement.

    WebXR raw ``local-floor`` data is never changed.  The transform creates a
    separately marked view used by downstream compatibility adapters.
    """

    yaw_degrees: float = 0.0
    translation_m: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not math.isfinite(float(self.yaw_degrees)):
            raise ValueError("WebXR calibration yaw must be finite")
        if len(self.translation_m) != 3 or not all(
            math.isfinite(float(value)) for value in self.translation_m
        ):
            raise ValueError("WebXR calibration translation must contain three finite meters")

    @property
    def is_identity(self) -> bool:
        return abs(float(self.yaw_degrees)) < 1e-12 and all(
            abs(float(value)) < 1e-12 for value in self.translation_m
        )

    def apply(self, frame: "WebXRHandFrame") -> "WebXRHandFrame":
        """Return a derived calibrated frame while leaving ``frame`` intact."""

        angle = math.radians(float(self.yaw_degrees))
        cosine = math.cos(angle)
        sine = math.sin(angle)
        half_sine = math.sin(angle * 0.5)
        half_cosine = math.cos(angle * 0.5)
        tx, ty, tz = (float(value) for value in self.translation_m)
        joints: list[WebXRJoint] = []
        for joint in frame.joints:
            position = joint.position
            orientation = joint.orientation
            if position is not None:
                x, y, z = position
                position = (
                    cosine * x + sine * z + tx,
                    y + ty,
                    -sine * x + cosine * z + tz,
                )
            if orientation is not None:
                x, y, z, w = orientation
                # Left-multiply the raw orientation by a world-Y yaw quaternion.
                orientation = (
                    half_cosine * x + half_sine * z,
                    half_cosine * y + half_sine * w,
                    half_cosine * z - half_sine * x,
                    half_cosine * w - half_sine * y,
                )
            joints.append(
                replace(
                    joint,
                    position=position,
                    orientation=orientation,
                    derived=True,
                )
            )
        coordinate_space = (
            f"derived({frame.coordinate_space};yaw_deg={float(self.yaw_degrees):g};"
            f"translation_m={tx:g},{ty:g},{tz:g})"
        )
        return replace(frame, joints=tuple(joints), coordinate_space=coordinate_space)


@dataclass(frozen=True)
class WebXRHandFrame:
    """One hand from one Quest XR frame, preserving the native 25-joint set."""

    joints: tuple[WebXRJoint, ...]
    hand_side: str
    tracked: bool
    timestamp_ms: int
    client_perf_ms: float | None
    predicted_display_time_ms: float | None
    sequence: int | None
    coordinate_space: str
    connection_state: str
    received_monotonic: float
    event: str = "hand_skeleton"

    @property
    def source_joint_count(self) -> int:
        return len(self.joints)

    @property
    def valid_joint_count(self) -> int:
        return sum(1 for joint in self.joints if joint.valid)

    def mediapipe_landmarks(self) -> tuple[tuple[float, float, float], ...]:
        """Return an explicit 25 -> 21 compatibility view, or empty on loss."""

        if not self.tracked or len(self.joints) != len(WEBXR_HAND_JOINT_NAMES):
            return ()
        selected: list[tuple[float, float, float]] = []
        for index in WEBXR_25_TO_MEDIAPIPE_21:
            joint = self.joints[index]
            if not joint.valid or joint.position is None:
                return ()
            selected.append(joint.position)
        return tuple(selected)

    def to_compat_frame(
        self,
        calibration: WebXRCalibrationTransform | None = None,
    ) -> CloudXRHandFrame:
        """Adapt to the application's existing 21-point latest-frame slot."""

        source = calibration.apply(self) if calibration is not None else self
        landmarks = source.mediapipe_landmarks()
        return CloudXRHandFrame(
            landmarks=landmarks,
            hand_side=source.hand_side,
            tracked=bool(source.tracked and landmarks),
            timestamp_ms=source.timestamp_ms,
            sequence=source.sequence,
            coordinate_space=source.coordinate_space,
            source_joint_count=25 if source.joints else 0,
            received_monotonic=source.received_monotonic,
        )


@dataclass(frozen=True)
class ParsedWebXRMessage:
    type: str
    sequence: int | None
    timestamp_ms: int
    handedness: str | None
    frames: tuple[WebXRHandFrame, ...]
    raw: Mapping[str, Any]


@dataclass(frozen=True)
class WebXRServerStatus:
    state: str  # stopped | listening | connected | receiving | warning | error
    message: str
    host: str
    port: int
    peer: str = ""
    connections: int = 0
    messages_received: int = 0
    frames_received: int = 0
    invalid_messages: int = 0
    last_event: str = ""
    quest_urls: tuple[str, ...] = ()
    effective_rate_hz: float = 0.0
    frame_interval_p95_ms: float = 0.0


def _finite_number(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise WebXRProtocolError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise WebXRProtocolError(f"{label} must be finite")
    return number


def _finite_tuple(value: Any, size: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise WebXRProtocolError(f"{label} must be an array of {size} numbers")
    if len(value) != size:
        raise WebXRProtocolError(f"{label} must contain {size} numbers")
    return tuple(_finite_number(component, label) for component in value)


def _optional_number(value: Any, label: str) -> float | None:
    if value is None:
        return None
    return _finite_number(value, label)


def _normalize_side(value: Any, *, allow_none: bool = False) -> str | None:
    if value is None and allow_none:
        return None
    side = str(value or "").strip().lower()
    if side in {"left", "right"}:
        return side
    raise WebXRProtocolError("handedness must be 'left' or 'right'")


def _int_or_none(value: Any, label: str) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise WebXRProtocolError(f"{label} must be an integer") from exc


def _timestamp_ms(message: Mapping[str, Any]) -> int:
    value = message.get("timestamp_ms")
    if value is None:
        return int(time.time() * 1000.0)
    number = _finite_number(value, "timestamp_ms")
    return int(number)


def _parse_joint(record: Any, expected_name: str) -> WebXRJoint:
    if not isinstance(record, Mapping):
        raise WebXRProtocolError(f"joint {expected_name!r} must be an object")
    name = str(record.get("name") or "").strip()
    if name != expected_name:
        raise WebXRProtocolError(
            f"expected joint {expected_name!r}, got {name!r}; WebXR order must be fixed"
        )
    valid = bool(record.get("valid", False))
    if not valid:
        return WebXRJoint(
            name=name,
            position=None,
            orientation=None,
            radius=_optional_number(record.get("radius"), f"{name}.radius"),
            valid=False,
        )
    position_raw = _finite_tuple(record.get("position"), 3, f"{name}.position")
    orientation_raw = _finite_tuple(record.get("orientation"), 4, f"{name}.orientation")
    radius = _optional_number(record.get("radius"), f"{name}.radius")
    if radius is not None and radius < 0.0:
        raise WebXRProtocolError(f"{name}.radius must be non-negative")
    return WebXRJoint(
        name=name,
        position=(position_raw[0], position_raw[1], position_raw[2]),
        orientation=(orientation_raw[0], orientation_raw[1], orientation_raw[2], orientation_raw[3]),
        radius=radius,
        valid=True,
    )


def _parse_hand(
    record: Any,
    *,
    sequence: int | None,
    timestamp_ms: int,
    client_perf_ms: float | None,
    predicted_display_time_ms: float | None,
    coordinate_space: str,
    connection_state: str,
) -> WebXRHandFrame:
    if not isinstance(record, Mapping):
        raise WebXRProtocolError("each hands entry must be an object")
    side = _normalize_side(record.get("handedness"))
    raw_joints = record.get("joints")
    if not isinstance(raw_joints, Sequence) or isinstance(raw_joints, (str, bytes, bytearray)):
        raise WebXRProtocolError(f"{side} hand joints must be an array")
    if len(raw_joints) != len(WEBXR_HAND_JOINT_NAMES):
        raise WebXRProtocolError(
            f"WebXR native joint set must contain exactly 25 joints, got {len(raw_joints)}"
        )
    joints = tuple(
        _parse_joint(raw_joint, expected_name)
        for raw_joint, expected_name in zip(raw_joints, WEBXR_HAND_JOINT_NAMES)
    )
    declared_tracked = bool(record.get("tracked", False))
    tracked = declared_tracked and all(joint.valid for joint in joints)
    return WebXRHandFrame(
        joints=joints,
        hand_side=str(side),
        tracked=tracked,
        timestamp_ms=timestamp_ms,
        client_perf_ms=client_perf_ms,
        predicted_display_time_ms=predicted_display_time_ms,
        sequence=sequence,
        coordinate_space=coordinate_space,
        connection_state=connection_state,
        received_monotonic=time.monotonic(),
    )


def parse_webxr_message(payload: str | bytes | bytearray | Mapping[str, Any]) -> ParsedWebXRMessage:
    """Validate one client message without changing its raw 25-joint data."""

    if isinstance(payload, Mapping):
        message = payload
    else:
        try:
            if isinstance(payload, (bytes, bytearray)):
                payload = bytes(payload).decode("utf-8")
            decoded = json.loads(payload)
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError) as exc:
            raise WebXRProtocolError("WebSocket message must be a UTF-8 JSON object") from exc
        if not isinstance(decoded, Mapping):
            raise WebXRProtocolError("WebSocket JSON root must be an object")
        message = decoded

    schema = message.get("schema")
    if schema not in {None, WEBXR_PROTOCOL_SCHEMA}:
        raise WebXRProtocolError(f"unsupported schema: {schema!r}")
    message_type = str(message.get("type") or "").strip().lower()
    allowed_types = {
        "client_hello",
        "hand_skeleton",
        "tracking_lost",
        "tracking_restored",
        "session_started",
        "session_ended",
        "client_error",
    }
    if message_type not in allowed_types:
        raise WebXRProtocolError(f"unsupported message type: {message_type!r}")

    sequence = _int_or_none(message.get("sequence"), "sequence")
    timestamp_ms = _timestamp_ms(message)
    side = _normalize_side(message.get("handedness"), allow_none=True)
    frames: tuple[WebXRHandFrame, ...] = ()

    if message_type == "hand_skeleton":
        joint_set = str(message.get("joint_set") or "").strip().lower()
        if joint_set != WEBXR_JOINT_SET:
            raise WebXRProtocolError(
                f"joint_set must be {WEBXR_JOINT_SET!r}; do not label WebXR data as OpenXR-26"
            )
        raw_hands = message.get("hands")
        if not isinstance(raw_hands, Sequence) or isinstance(raw_hands, (str, bytes, bytearray)):
            raise WebXRProtocolError("hand_skeleton.hands must be an array")
        client_perf_ms = _optional_number(message.get("client_perf_ms"), "client_perf_ms")
        predicted_display_time_ms = _optional_number(
            message.get("xr_predicted_display_time_ms"),
            "xr_predicted_display_time_ms",
        )
        coordinate_space = str(message.get("coordinate_space") or "webxr_local_floor").strip().lower()
        connection_state = str(message.get("connection_state") or "unknown").strip().lower()
        parsed_frames = [
            _parse_hand(
                record,
                sequence=sequence,
                timestamp_ms=timestamp_ms,
                client_perf_ms=client_perf_ms,
                predicted_display_time_ms=predicted_display_time_ms,
                coordinate_space=coordinate_space,
                connection_state=connection_state,
            )
            for record in raw_hands
        ]
        sides = [frame.hand_side for frame in parsed_frames]
        if len(sides) != len(set(sides)):
            raise WebXRProtocolError("hand_skeleton contains duplicate handedness entries")
        frames = tuple(parsed_frames)

    return ParsedWebXRMessage(
        type=message_type,
        sequence=sequence,
        timestamp_ms=timestamp_ms,
        handedness=side,
        frames=frames,
        raw=message,
    )


def derive_openxr26_joints(frame: WebXRHandFrame) -> tuple[WebXRJoint, ...]:
    """Return ``derived palm + WebXR25`` for compatibility, clearly marked.

    The palm position is the centroid of wrist plus the five metacarpals.  Its
    orientation follows the wrist.  The raw frame remains unchanged.
    """

    if len(frame.joints) != len(WEBXR_HAND_JOINT_NAMES):
        return frame.joints
    source_names = {
        "wrist",
        "thumb-metacarpal",
        "index-finger-metacarpal",
        "middle-finger-metacarpal",
        "ring-finger-metacarpal",
        "pinky-finger-metacarpal",
    }
    sources = [
        joint
        for joint in frame.joints
        if joint.name in source_names and joint.valid and joint.position is not None
    ]
    wrist = frame.joints[0]
    valid = len(sources) == len(source_names) and wrist.orientation is not None
    if valid:
        count = float(len(sources))
        position = tuple(sum(joint.position[axis] for joint in sources) / count for axis in range(3))
        radii = [joint.radius for joint in sources if joint.radius is not None]
        radius = sum(radii) / len(radii) if radii else None
        palm = WebXRJoint(
            name="palm",
            position=(position[0], position[1], position[2]),
            orientation=wrist.orientation,
            radius=radius,
            valid=True,
            derived=True,
        )
    else:
        palm = WebXRJoint(
            name="palm",
            position=None,
            orientation=None,
            radius=None,
            valid=False,
            derived=True,
        )
    return (palm, *frame.joints)


def local_ipv4_addresses() -> tuple[str, ...]:
    """Best-effort list of non-loopback local IPv4 addresses."""

    candidates: set[str] = set()
    hostnames = {socket.gethostname(), socket.getfqdn()}
    for hostname in hostnames:
        try:
            candidates.update(socket.gethostbyname_ex(hostname)[2])
        except OSError:
            pass
        try:
            for family, _type, _proto, _canonname, sockaddr in socket.getaddrinfo(
                hostname, None, socket.AF_INET
            ):
                if family == socket.AF_INET:
                    candidates.add(sockaddr[0])
        except OSError:
            pass
    for target in (("8.8.8.8", 80), ("1.1.1.1", 80)):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.connect(target)
                candidates.add(probe.getsockname()[0])
        except OSError:
            pass
    valid = []
    for candidate in candidates:
        try:
            address = ipaddress.ip_address(candidate)
        except ValueError:
            continue
        if address.version == 4 and not address.is_loopback and not address.is_link_local:
            valid.append(str(address))
    return tuple(sorted(set(valid), key=lambda value: tuple(int(part) for part in value.split("."))))


def _certificate_has_hosts(cert_path: Path, hosts: set[str]) -> bool:
    try:
        from cryptography import x509
        from cryptography.x509.oid import ExtensionOID

        cert = x509.load_pem_x509_certificate(cert_path.read_bytes())
        if cert.not_valid_after_utc <= datetime.now(timezone.utc) + timedelta(days=7):
            return False
        extension = cert.extensions.get_extension_for_oid(ExtensionOID.SUBJECT_ALTERNATIVE_NAME)
        names = extension.value
        dns_names = set(names.get_values_for_type(x509.DNSName))
        ip_names = {str(value) for value in names.get_values_for_type(x509.IPAddress)}
        return hosts.issubset(dns_names | ip_names)
    except Exception:
        return False


def ensure_webxr_certificate(
    cert_path: str | os.PathLike[str] = DEFAULT_WEBXR_CERT,
    key_path: str | os.PathLike[str] = DEFAULT_WEBXR_KEY,
    *,
    advertise_hosts: Sequence[str] = (),
) -> tuple[Path, Path, str]:
    """Create/reuse a LAN TLS certificate signed by a small local CA.

    For generated certificates, ``quest_webxr_ca.crt`` is placed beside the
    server certificate.  Importing that CA on Quest avoids treating the WebXR
    page as an insecure context.  A caller-provided valid certificate/key pair
    is reused as-is.
    """

    cert = Path(cert_path).expanduser().resolve()
    key = Path(key_path).expanduser().resolve()
    hosts = {"localhost", socket.gethostname(), socket.getfqdn(), "127.0.0.1"}
    hosts.update(local_ipv4_addresses())
    hosts.update(str(host).strip() for host in advertise_hosts if str(host).strip())
    hosts.discard("0.0.0.0")

    if cert.is_file() and key.is_file() and _certificate_has_hosts(cert, hosts):
        fingerprint = hashlib.sha256(ssl.PEM_cert_to_DER_cert(cert.read_text(encoding="ascii"))).hexdigest()
        return cert, key, fingerprint

    try:
        from cryptography import x509
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import rsa
        from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID
    except ImportError as exc:
        raise RuntimeError(
            "cryptography is required to generate the WebXR TLS certificate; "
            "install it or provide WEBXR_TLS_CERT/WEBXR_TLS_KEY"
        ) from exc

    cert.parent.mkdir(parents=True, exist_ok=True)
    key.parent.mkdir(parents=True, exist_ok=True)
    ca_cert_path = cert.with_name(DEFAULT_WEBXR_CA_CERT.name)
    ca_key_path = key.with_name(DEFAULT_WEBXR_CA_KEY.name)
    ca_private_key = None
    ca_certificate = None
    if ca_cert_path.is_file() and ca_key_path.is_file():
        try:
            ca_certificate = x509.load_pem_x509_certificate(ca_cert_path.read_bytes())
            ca_private_key = serialization.load_pem_private_key(
                ca_key_path.read_bytes(), password=None
            )
            if ca_certificate.not_valid_after_utc <= datetime.now(timezone.utc) + timedelta(days=30):
                ca_certificate = None
                ca_private_key = None
        except Exception:
            ca_certificate = None
            ca_private_key = None

    now = datetime.now(timezone.utc)
    if ca_certificate is None or ca_private_key is None:
        ca_private_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        ca_subject = x509.Name(
            [x509.NameAttribute(NameOID.COMMON_NAME, "TsenseI Quest WebXR Local CA")]
        )
        ca_certificate = (
            x509.CertificateBuilder()
            .subject_name(ca_subject)
            .issuer_name(ca_subject)
            .public_key(ca_private_key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - timedelta(days=1))
            .not_valid_after(now + timedelta(days=3650))
            .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
            .add_extension(
                x509.KeyUsage(
                    digital_signature=True,
                    content_commitment=False,
                    key_encipherment=False,
                    data_encipherment=False,
                    key_agreement=False,
                    key_cert_sign=True,
                    crl_sign=True,
                    encipher_only=False,
                    decipher_only=False,
                ),
                critical=True,
            )
            .add_extension(
                x509.SubjectKeyIdentifier.from_public_key(ca_private_key.public_key()),
                critical=False,
            )
            .sign(ca_private_key, hashes.SHA256())
        )
        ca_key_path.write_bytes(
            ca_private_key.private_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PrivateFormat.PKCS8,
                encryption_algorithm=serialization.NoEncryption(),
            )
        )
        ca_cert_path.write_bytes(ca_certificate.public_bytes(serialization.Encoding.PEM))

    private_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    common_name = next(
        (host for host in advertise_hosts if str(host).strip() and str(host).strip() != "0.0.0.0"),
        "Quest WebXR Hand Skeleton",
    )
    subject = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, str(common_name))])
    san_entries = []
    for host in sorted(hosts):
        try:
            san_entries.append(x509.IPAddress(ipaddress.ip_address(host)))
        except ValueError:
            san_entries.append(x509.DNSName(host))
    certificate = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(ca_certificate.subject)
        .public_key(private_key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - timedelta(days=1))
        .not_valid_after(now + timedelta(days=825))
        .add_extension(x509.SubjectAlternativeName(san_entries), critical=False)
        .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
        .add_extension(
            x509.KeyUsage(
                digital_signature=True,
                content_commitment=False,
                key_encipherment=True,
                data_encipherment=False,
                key_agreement=False,
                key_cert_sign=False,
                crl_sign=False,
                encipher_only=False,
                decipher_only=False,
            ),
            critical=True,
        )
        .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
        .add_extension(
            x509.AuthorityKeyIdentifier.from_issuer_public_key(ca_private_key.public_key()),
            critical=False,
        )
        .sign(ca_private_key, hashes.SHA256())
    )
    key.write_bytes(
        private_key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.PKCS8,
            encryption_algorithm=serialization.NoEncryption(),
        )
    )
    cert.write_bytes(certificate.public_bytes(serialization.Encoding.PEM))
    fingerprint = certificate.fingerprint(hashes.SHA256()).hex()
    return cert, key, fingerprint


@dataclass
class _ConnectionState:
    connection_id: int
    peer: str
    sock: socket.socket
    connected_at: float
    last_message_at: float
    last_seen_by_side: dict[str, float]
    tracked_sides: set[str]


class _WebXRHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    owner: "QuestWebXRServer"


def _read_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise EOFError("WebSocket peer closed")
        chunks.extend(chunk)
    return bytes(chunks)


def _encode_server_frame(opcode: int, payload: bytes = b"") -> bytes:
    first = 0x80 | (opcode & 0x0F)
    length = len(payload)
    if length < 126:
        header = bytes((first, length))
    elif length <= 0xFFFF:
        header = bytes((first, 126)) + struct.pack("!H", length)
    else:
        header = bytes((first, 127)) + struct.pack("!Q", length)
    return header + payload


def _read_client_frame(sock: socket.socket) -> tuple[bool, int, bytes]:
    header = _read_exact(sock, 2)
    first, second = header
    final = bool(first & 0x80)
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    if not masked:
        raise WebXRProtocolError("client WebSocket frames must be masked")
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", _read_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", _read_exact(sock, 8))[0]
    if length > MAX_WEBSOCKET_MESSAGE_BYTES:
        raise WebXRProtocolError("WebSocket message exceeds 1 MiB limit")
    mask = _read_exact(sock, 4)
    payload = bytearray(_read_exact(sock, length))
    for index in range(length):
        payload[index] ^= mask[index & 3]
    return final, opcode, bytes(payload)


class _WebXRRequestHandler(BaseHTTPRequestHandler):
    server: _WebXRHTTPServer
    protocol_version = "HTTP/1.1"

    def log_message(self, _format: str, *_args: object) -> None:
        return

    def _send_bytes(
        self,
        status: HTTPStatus,
        body: bytes,
        *,
        content_type: str,
        extra_headers: Mapping[str, str] | None = None,
    ) -> None:
        self.send_response(int(status))
        self.send_header("content-type", content_type)
        self.send_header("content-length", str(len(body)))
        self.send_header("cache-control", "no-store")
        self.send_header("x-content-type-options", "nosniff")
        if extra_headers:
            for name, value in extra_headers.items():
                self.send_header(name, value)
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, status: HTTPStatus, payload: Mapping[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self._send_bytes(status, body, content_type="application/json; charset=utf-8")

    def do_GET(self) -> None:  # noqa: N802
        path = urlsplit(self.path).path
        if path in {"/", "/index.html"}:
            try:
                page = _PAGE_PATH.read_bytes()
            except OSError as exc:
                self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})
                return
            self._send_bytes(
                HTTPStatus.OK,
                page,
                content_type="text/html; charset=utf-8",
                extra_headers={
                    "permissions-policy": "xr-spatial-tracking=(self)",
                    "content-security-policy": (
                        "default-src 'self'; script-src 'self' 'unsafe-inline'; "
                        "style-src 'self' 'unsafe-inline'; connect-src 'self' wss:"
                    ),
                },
            )
            return
        if path == "/healthz":
            self._send_json(HTTPStatus.OK, self.server.owner.health_snapshot())
            return
        if path == WEBXR_PATH:
            if self.headers.get("upgrade", "").strip().lower() != "websocket":
                self._send_json(
                    HTTPStatus.UPGRADE_REQUIRED,
                    {"error": "WebSocket upgrade required", "path": WEBXR_PATH},
                )
                return
            self._serve_websocket()
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def _serve_websocket(self) -> None:
        owner = self.server.owner
        origin = self.headers.get("origin")
        host_header = self.headers.get("host", "")
        if origin:
            try:
                if urlsplit(origin).netloc.lower() != host_header.lower():
                    self._send_json(HTTPStatus.FORBIDDEN, {"error": "cross-origin WSS rejected"})
                    return
            except ValueError:
                self._send_json(HTTPStatus.FORBIDDEN, {"error": "invalid Origin"})
                return
        version = self.headers.get("sec-websocket-version", "")
        key = self.headers.get("sec-websocket-key", "").strip()
        if version != "13" or not key:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": "invalid WebSocket handshake"})
            return
        accept = base64.b64encode(hashlib.sha1((key + _WEBSOCKET_GUID).encode("ascii")).digest()).decode("ascii")
        self.send_response_only(HTTPStatus.SWITCHING_PROTOCOLS)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.wfile.flush()
        self.close_connection = True

        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        connection_id = owner._register_connection(peer, self.connection)
        self.connection.sendall(
            _encode_server_frame(
                0x1,
                json.dumps(
                    {
                        "schema": WEBXR_PROTOCOL_SCHEMA,
                        "type": "server_ready",
                        "joint_set": WEBXR_JOINT_SET,
                        "timestamp_ms": int(time.time() * 1000.0),
                    },
                    separators=(",", ":"),
                ).encode("utf-8"),
            )
        )
        fragmented_opcode: int | None = None
        fragmented = bytearray()
        try:
            while owner.running:
                final, opcode, payload = _read_client_frame(self.connection)
                if opcode == 0x8:
                    try:
                        self.connection.sendall(_encode_server_frame(0x8, payload[:125]))
                    except OSError:
                        pass
                    break
                if opcode == 0x9:
                    self.connection.sendall(_encode_server_frame(0xA, payload[:125]))
                    continue
                if opcode == 0xA:
                    continue
                if opcode in {0x1, 0x2}:
                    if fragmented_opcode is not None:
                        raise WebXRProtocolError("new data frame before fragmented message completed")
                    if final:
                        if opcode != 0x1:
                            raise WebXRProtocolError("binary skeleton messages are not supported")
                        owner._handle_ws_text(connection_id, payload.decode("utf-8"))
                    else:
                        fragmented_opcode = opcode
                        fragmented.extend(payload)
                    continue
                if opcode == 0x0:
                    if fragmented_opcode is None:
                        raise WebXRProtocolError("unexpected continuation frame")
                    fragmented.extend(payload)
                    if len(fragmented) > MAX_WEBSOCKET_MESSAGE_BYTES:
                        raise WebXRProtocolError("fragmented message exceeds 1 MiB limit")
                    if final:
                        if fragmented_opcode != 0x1:
                            raise WebXRProtocolError("binary skeleton messages are not supported")
                        owner._handle_ws_text(connection_id, bytes(fragmented).decode("utf-8"))
                        fragmented_opcode = None
                        fragmented.clear()
                    continue
                raise WebXRProtocolError(f"unsupported WebSocket opcode: {opcode}")
        except (EOFError, ConnectionError, OSError, UnicodeDecodeError):
            pass
        except Exception as exc:
            owner._record_protocol_error(connection_id, str(exc))
            try:
                reason = str(exc).encode("utf-8")[:120]
                self.connection.sendall(_encode_server_frame(0x8, struct.pack("!H", 1002) + reason))
            except OSError:
                pass
        finally:
            owner._unregister_connection(connection_id, reason="websocket_disconnected")


class QuestWebXRServer:
    """Stoppable same-origin HTTPS/WSS service for Quest WebXR hands."""

    def __init__(
        self,
        *,
        on_frame: Callable[[WebXRHandFrame], None],
        on_status: Callable[[WebXRServerStatus], None] | None = None,
    ) -> None:
        self._on_frame = on_frame
        self._on_status = on_status
        self._lock = threading.RLock()
        self._server: _WebXRHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._monitor_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._host = DEFAULT_WEBXR_HOST
        self._port = DEFAULT_WEBXR_PORT
        self._quest_urls: tuple[str, ...] = ()
        self._connections: dict[int, _ConnectionState] = {}
        self._next_connection_id = 1
        self._messages_received = 0
        self._frames_received = 0
        self._invalid_messages = 0
        self._last_event = ""
        self._last_status_at = 0.0
        self._last_skeleton_at = 0.0
        self._frame_intervals_ms: deque[float] = deque(maxlen=512)
        self._latest_frames: dict[str, WebXRHandFrame] = {}
        self._jsonl_path: Path | None = None
        self._jsonl_lock = threading.Lock()
        self._cert_fingerprint = ""

    @property
    def running(self) -> bool:
        with self._lock:
            return self._server is not None and self._thread is not None and self._thread.is_alive()

    @property
    def endpoint(self) -> tuple[str, int]:
        with self._lock:
            return self._host, self._port

    @property
    def quest_urls(self) -> tuple[str, ...]:
        with self._lock:
            return self._quest_urls

    def latest_frame(self, hand_side: str) -> WebXRHandFrame | None:
        side = str(hand_side).strip().lower()
        with self._lock:
            return self._latest_frames.get(side)

    def start(
        self,
        host: str = DEFAULT_WEBXR_HOST,
        port: int = DEFAULT_WEBXR_PORT,
        *,
        cert_path: str | os.PathLike[str] = DEFAULT_WEBXR_CERT,
        key_path: str | os.PathLike[str] = DEFAULT_WEBXR_KEY,
        advertise_hosts: Sequence[str] = (),
        jsonl_path: str | os.PathLike[str] | None = None,
        append_jsonl: bool = False,
    ) -> tuple[str, int]:
        self.stop(notify=False)
        bind_host = str(host or DEFAULT_WEBXR_HOST).strip()
        bind_port = int(port)
        if not (0 <= bind_port <= 65535):
            raise ValueError("WebXR HTTPS/WSS port must be between 0 and 65535")

        advertised = [str(value).strip() for value in advertise_hosts if str(value).strip()]
        if bind_host not in {"0.0.0.0", "127.0.0.1", "localhost"}:
            advertised.append(bind_host)
        cert, key, fingerprint = ensure_webxr_certificate(
            cert_path,
            key_path,
            advertise_hosts=advertised,
        )
        server = _WebXRHTTPServer((bind_host, bind_port), _WebXRRequestHandler)
        server.owner = self
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(certfile=str(cert), keyfile=str(key))
        server.socket = context.wrap_socket(server.socket, server_side=True)
        bound_host, bound_port = server.server_address[:2]

        url_hosts = set(advertised)
        url_hosts.update(local_ipv4_addresses())
        if not url_hosts:
            url_hosts.add("127.0.0.1")
        urls = tuple(f"https://{value}:{int(bound_port)}/" for value in sorted(url_hosts))
        output_path = Path(jsonl_path).expanduser().resolve() if jsonl_path else None
        if output_path is not None:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            if not append_jsonl:
                output_path.write_text("", encoding="utf-8")

        stop_event = threading.Event()
        with self._lock:
            self._server = server
            self._host = str(bound_host)
            self._port = int(bound_port)
            self._quest_urls = urls
            self._connections.clear()
            self._next_connection_id = 1
            self._messages_received = 0
            self._frames_received = 0
            self._invalid_messages = 0
            self._last_event = "listening"
            self._last_skeleton_at = 0.0
            self._frame_intervals_ms.clear()
            self._latest_frames.clear()
            self._jsonl_path = output_path
            self._cert_fingerprint = fingerprint
            self._stop_event = stop_event
            self._thread = threading.Thread(
                target=server.serve_forever,
                kwargs={"poll_interval": 0.1},
                name="QuestWebXR-HTTPS-WSS",
                daemon=True,
            )
            self._monitor_thread = threading.Thread(
                target=self._monitor_stale_tracking,
                args=(stop_event,),
                name="QuestWebXR-StaleMonitor",
                daemon=True,
            )
            thread = self._thread
            monitor = self._monitor_thread
        thread.start()
        monitor.start()
        self._emit_status("listening", f"HTTPS/WSS {bound_host}:{bound_port}")
        return str(bound_host), int(bound_port)

    def stop(self, *, notify: bool = True) -> None:
        with self._lock:
            server = self._server
            thread = self._thread
            monitor = self._monitor_thread
            connections = list(self._connections.values())
            stop_event = self._stop_event
            was_running = server is not None or thread is not None
            self._server = None
            self._thread = None
            self._monitor_thread = None
            stop_event.set()
        # Clear application tracking before removing connection state.  This
        # guarantees a final loss frame even when the service itself is what
        # closes an otherwise healthy Quest connection.
        self._clear_all_tracking("server_stopped")
        with self._lock:
            self._connections.clear()
        for state in connections:
            try:
                state.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                state.sock.close()
            except OSError:
                pass
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        if monitor is not None and monitor is not threading.current_thread():
            monitor.join(timeout=1.0)
        if notify and was_running:
            self._emit_status("stopped", "WebXR 服务已停止")

    def health_snapshot(self) -> dict[str, Any]:
        with self._lock:
            rate_hz, interval_p95_ms = self._receive_timing_locked()
            return {
                "status": "ok" if self.running else "stopped",
                "service": "quest-webxr-hand-skeleton",
                "schema": WEBXR_PROTOCOL_SCHEMA,
                "joint_set": WEBXR_JOINT_SET,
                "native_joint_count": 25,
                "https_port": self._port,
                "websocket_path": WEBXR_PATH,
                "connections": len(self._connections),
                "messages_received": self._messages_received,
                "frames_received": self._frames_received,
                "invalid_messages": self._invalid_messages,
                "effective_rate_hz": rate_hz,
                "frame_interval_p95_ms": interval_p95_ms,
                "timing_sample_count": len(self._frame_intervals_ms),
                "last_event": self._last_event,
                "tracked_hands": {
                    side: frame.tracked for side, frame in self._latest_frames.items()
                },
                "quest_urls": list(self._quest_urls),
                "certificate_sha256": self._cert_fingerprint,
                "server_time_ms": int(time.time() * 1000.0),
            }

    def _emit_status(self, state: str, message: str, *, peer: str = "") -> None:
        callback = self._on_status
        if callback is None:
            return
        with self._lock:
            rate_hz, interval_p95_ms = self._receive_timing_locked()
            status = WebXRServerStatus(
                state=state,
                message=message,
                host=self._host,
                port=self._port,
                peer=peer,
                connections=len(self._connections),
                messages_received=self._messages_received,
                frames_received=self._frames_received,
                invalid_messages=self._invalid_messages,
                last_event=self._last_event,
                quest_urls=self._quest_urls,
                effective_rate_hz=rate_hz,
                frame_interval_p95_ms=interval_p95_ms,
            )
        try:
            callback(status)
        except Exception:
            pass

    def _register_connection(self, peer: str, sock: socket.socket) -> int:
        now = time.monotonic()
        with self._lock:
            if not self._connections:
                # Timing acceptance is per live Quest session; an intentional
                # reconnect gap must not inflate the frame-interval P95.
                self._last_skeleton_at = 0.0
                self._frame_intervals_ms.clear()
            connection_id = self._next_connection_id
            self._next_connection_id += 1
            self._connections[connection_id] = _ConnectionState(
                connection_id=connection_id,
                peer=peer,
                sock=sock,
                connected_at=now,
                last_message_at=now,
                last_seen_by_side={},
                tracked_sides=set(),
            )
            self._last_event = "websocket_connected"
        self._emit_status("connected", "Quest WSS 已连接", peer=peer)
        return connection_id

    def _unregister_connection(self, connection_id: int, *, reason: str) -> None:
        with self._lock:
            state = self._connections.pop(connection_id, None)
            self._last_event = reason
        if state is None:
            return
        for side in tuple(state.tracked_sides):
            self._emit_tracking_lost(side, state, reason)
        with self._lock:
            has_connections = bool(self._connections)
        next_state = "connected" if has_connections else "listening"
        message = "仍有 Quest 连接" if has_connections else "等待 Quest 连接"
        self._emit_status(next_state, message, peer=state.peer)

    def _record_protocol_error(self, connection_id: int, message: str) -> None:
        with self._lock:
            self._invalid_messages += 1
            state = self._connections.get(connection_id)
            peer = state.peer if state is not None else ""
            self._last_event = "protocol_error"
        self._emit_status("warning", f"WebXR 报文错误: {message}", peer=peer)

    def _handle_ws_text(self, connection_id: int, payload: str) -> None:
        try:
            parsed = parse_webxr_message(payload)
        except Exception as exc:
            self._record_protocol_error(connection_id, str(exc))
            return
        now = time.monotonic()
        with self._lock:
            state = self._connections.get(connection_id)
            if state is None:
                return
            state.last_message_at = now
            self._messages_received += 1
            self._last_event = parsed.type
        self._append_jsonl(parsed.raw, peer=state.peer, received_monotonic=now)

        if parsed.type == "hand_skeleton":
            with self._lock:
                self._frames_received += 1
                if self._last_skeleton_at > 0.0:
                    interval_ms = (now - self._last_skeleton_at) * 1000.0
                    if 0.0 < interval_ms < 5000.0:
                        self._frame_intervals_ms.append(interval_ms)
                self._last_skeleton_at = now
            for frame in parsed.frames:
                with self._lock:
                    state.last_seen_by_side[frame.hand_side] = now
                    self._latest_frames[frame.hand_side] = frame
                    was_tracked = frame.hand_side in state.tracked_sides
                    if frame.tracked:
                        state.tracked_sides.add(frame.hand_side)
                    else:
                        state.tracked_sides.discard(frame.hand_side)
                if frame.tracked:
                    self._dispatch_frame(frame)
                elif was_tracked:
                    self._dispatch_frame(replace(frame, event="tracking_lost"))

            if now - self._last_status_at >= 1.0:
                self._last_status_at = now
                tracked = sorted(state.tracked_sides)
                summary = ",".join(tracked) if tracked else "无完整手"
                with self._lock:
                    rate_hz, p95_ms = self._receive_timing_locked()
                self._emit_status(
                    "receiving",
                    f"接收中 · {rate_hz:.1f}Hz · P95 {p95_ms:.1f}ms · {summary}",
                    peer=state.peer,
                )
            return

        if parsed.type == "tracking_lost" and parsed.handedness:
            self._emit_tracking_lost(parsed.handedness, state, "client_tracking_lost")
        elif parsed.type == "session_ended":
            for side in tuple(state.tracked_sides):
                self._emit_tracking_lost(side, state, "client_session_ended")
        elif parsed.type == "client_error":
            self._emit_status("warning", "Quest 页面报告错误", peer=state.peer)
        elif parsed.type in {"session_started", "tracking_restored", "client_hello"}:
            self._emit_status("connected", parsed.type, peer=state.peer)

    def _emit_tracking_lost(self, side: str, state: _ConnectionState, reason: str) -> None:
        with self._lock:
            was_tracked = side in state.tracked_sides
            state.tracked_sides.discard(side)
            # A second browser tab/session may still own a live pose for this
            # hand.  Disconnecting one producer must not clear the other.
            tracked_elsewhere = any(
                other is not state and side in other.tracked_sides
                for other in self._connections.values()
            )
            if tracked_elsewhere:
                return
            previous = self._latest_frames.get(side)
            if not was_tracked and previous is not None and not previous.tracked:
                return
            frame = WebXRHandFrame(
                joints=(),
                hand_side=side,
                tracked=False,
                timestamp_ms=int(time.time() * 1000.0),
                client_perf_ms=None,
                predicted_display_time_ms=None,
                sequence=previous.sequence if previous is not None else None,
                coordinate_space=(
                    previous.coordinate_space if previous is not None else "webxr_local_floor"
                ),
                connection_state="disconnected" if "disconnect" in reason else "connected",
                received_monotonic=time.monotonic(),
                event="tracking_lost",
            )
            self._latest_frames[side] = frame
            self._last_event = "tracking_lost"
        self._dispatch_frame(frame)

    def _clear_all_tracking(self, reason: str) -> None:
        with self._lock:
            states = list(self._connections.values())
        for state in states:
            for side in tuple(state.tracked_sides):
                self._emit_tracking_lost(side, state, reason)

    def _dispatch_frame(self, frame: WebXRHandFrame) -> None:
        try:
            self._on_frame(frame)
        except Exception as exc:
            self._emit_status("error", f"分发 WebXR 数据失败: {exc}")

    def _receive_timing_locked(self) -> tuple[float, float]:
        """Return recent host receive rate/P95 while ``self._lock`` is held."""

        if not self._frame_intervals_ms:
            return 0.0, 0.0
        values = tuple(self._frame_intervals_ms)
        mean_ms = sum(values) / len(values)
        ordered = sorted(values)
        p95_index = max(0, math.ceil(len(ordered) * 0.95) - 1)
        return 1000.0 / mean_ms, ordered[p95_index]

    def _monitor_stale_tracking(self, stop_event: threading.Event) -> None:
        while not stop_event.wait(0.1):
            now = time.monotonic()
            stale: list[tuple[str, _ConnectionState]] = []
            with self._lock:
                for state in self._connections.values():
                    for side in tuple(state.tracked_sides):
                        last_seen = state.last_seen_by_side.get(side, state.last_message_at)
                        if now - last_seen >= WEBXR_STALE_TIMEOUT_S:
                            stale.append((side, state))
            for side, state in stale:
                self._emit_tracking_lost(side, state, "tracking_timeout")

    def _append_jsonl(
        self,
        raw: Mapping[str, Any],
        *,
        peer: str,
        received_monotonic: float,
    ) -> None:
        path = self._jsonl_path
        if path is None:
            return
        record = dict(raw)
        record["host_received_wall_time_ms"] = int(time.time() * 1000.0)
        record["host_received_monotonic_s"] = received_monotonic
        record["host_peer"] = peer
        line = json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
        try:
            with self._jsonl_lock:
                with path.open("a", encoding="utf-8") as stream:
                    stream.write(line)
        except OSError as exc:
            self._emit_status("warning", f"JSONL 写入失败: {exc}", peer=peer)


__all__ = [
    "DEFAULT_WEBXR_CA_CERT",
    "DEFAULT_WEBXR_CA_KEY",
    "DEFAULT_WEBXR_CERT",
    "DEFAULT_WEBXR_HOST",
    "DEFAULT_WEBXR_KEY",
    "DEFAULT_WEBXR_PORT",
    "ParsedWebXRMessage",
    "QuestWebXRServer",
    "WEBXR_25_TO_MEDIAPIPE_21",
    "WEBXR_HAND_JOINT_NAMES",
    "WEBXR_JOINT_SET",
    "WEBXR_PATH",
    "WEBXR_PROTOCOL_SCHEMA",
    "WebXRHandFrame",
    "WebXRCalibrationTransform",
    "WebXRJoint",
    "WebXRProtocolError",
    "WebXRServerStatus",
    "derive_openxr26_joints",
    "ensure_webxr_certificate",
    "local_ipv4_addresses",
    "parse_webxr_message",
]
