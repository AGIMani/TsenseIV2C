from __future__ import annotations

from dataclasses import dataclass, field
import dataclasses
from pathlib import Path
from typing import Any

import yaml

MEDIAPIPE_HANDS_21 = "mediapipe_hands_21"
EXPECTED_KEYPOINT_COUNT = 21


@dataclass(frozen=True)
class RobotKeypoint:
    id: int
    source_name: str
    semantic: str
    finger: str
    link: str
    local_xyz: tuple[float, float, float]
    parent: str | None
    child: str | None
    type: str
    weight: float = 1.0
    annotation_status: str = "auto_initialized"
    note: str | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RobotKeypoint":
        xyz = data.get("local_xyz")
        if not isinstance(xyz, list | tuple) or len(xyz) != 3:
            raise ValueError(f"keypoint {data.get('id')} local_xyz must have length 3")
        return cls(
            id=int(data["id"]),
            source_name=str(data["source_name"]),
            semantic=str(data["semantic"]),
            finger=str(data["finger"]),
            link=str(data["link"]),
            local_xyz=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
            parent=data.get("parent"),
            child=data.get("child"),
            type=str(data["type"]),
            weight=float(data.get("weight", 1.0)),
            annotation_status=str(data.get("annotation_status", "auto_initialized")),
            note=data.get("note"),
        )

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "id": self.id,
            "source_name": self.source_name,
            "semantic": self.semantic,
            "finger": self.finger,
            "link": self.link,
            "local_xyz": [float(v) for v in self.local_xyz],
            "parent": self.parent,
            "child": self.child,
            "type": self.type,
            "weight": self.weight,
            "annotation_status": self.annotation_status,
        }
        if self.note:
            out["note"] = self.note
        return out


@dataclass
class RobotKeypointSet:
    schema_version: int
    source_skeleton: str
    robot_name: str
    urdf_path: str
    unit: str
    base_link: str
    hand_side: str
    keypoints: list[RobotKeypoint]
    quality_status: str = "Draft"
    annotation_version: int = 1
    urdf_sha256: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RobotKeypointSet":
        keypoints = [RobotKeypoint.from_dict(item) for item in data.get("keypoints", [])]
        known = {
            "schema_version",
            "source_skeleton",
            "robot_name",
            "urdf_path",
            "unit",
            "base_link",
            "hand_side",
            "keypoints",
            "quality_status",
            "annotation_version",
            "urdf_sha256",
        }
        metadata = {k: v for k, v in data.items() if k not in known}
        result = cls(
            schema_version=int(data.get("schema_version", 1)),
            source_skeleton=str(data.get("source_skeleton", MEDIAPIPE_HANDS_21)),
            robot_name=str(data["robot_name"]),
            urdf_path=str(data["urdf_path"]),
            unit=str(data.get("unit", "meter")),
            base_link=str(data["base_link"]),
            hand_side=str(data.get("hand_side", "unknown")),
            keypoints=keypoints,
            quality_status=str(data.get("quality_status", "Draft")),
            annotation_version=int(data.get("annotation_version", 1)),
            urdf_sha256=data.get("urdf_sha256"),
            metadata=metadata,
        )
        result.validate()
        return result

    @classmethod
    def load(cls, path: str | Path) -> "RobotKeypointSet":
        return cls.from_dict(yaml.safe_load(Path(path).read_text(encoding="utf-8")))

    def validate(self) -> None:
        if len(self.keypoints) != EXPECTED_KEYPOINT_COUNT:
            raise ValueError(
                f"expected {EXPECTED_KEYPOINT_COUNT} MediaPipe keypoints, got {len(self.keypoints)}"
            )
        ids = sorted(kp.id for kp in self.keypoints)
        if ids != list(range(EXPECTED_KEYPOINT_COUNT)):
            raise ValueError(f"keypoint ids must be contiguous 0..{EXPECTED_KEYPOINT_COUNT - 1}, got {ids}")
        if self.source_skeleton != MEDIAPIPE_HANDS_21:
            raise ValueError(
                f"source_skeleton must be {MEDIAPIPE_HANDS_21!r}, got {self.source_skeleton!r}"
            )

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "schema_version": self.schema_version,
            "source_skeleton": self.source_skeleton,
            "robot_name": self.robot_name,
            "urdf_path": self.urdf_path,
            "unit": self.unit,
            "base_link": self.base_link,
            "hand_side": self.hand_side,
            "quality_status": self.quality_status,
            "annotation_version": self.annotation_version,
        }
        if self.urdf_sha256:
            out["urdf_sha256"] = self.urdf_sha256
        out.update(self.metadata)
        out["keypoints"] = [kp.to_dict() for kp in sorted(self.keypoints, key=lambda item: item.id)]
        return out

    def save(self, path: str | Path) -> None:
        Path(path).write_text(yaml.safe_dump(self.to_dict(), sort_keys=False, allow_unicode=True), encoding="utf-8")

    def by_id(self) -> dict[int, RobotKeypoint]:
        return {kp.id: kp for kp in self.keypoints}

    def by_name(self) -> dict[str, RobotKeypoint]:
        return {kp.source_name: kp for kp in self.keypoints}

    def skeleton_edges(self) -> list[tuple[str, str]]:
        by_name = self.by_name()
        edges: set[tuple[str, str]] = set()
        for kp in self.keypoints:
            if kp.parent in by_name:
                edges.add((str(kp.parent), kp.source_name))
            if kp.child in by_name:
                edges.add((kp.source_name, str(kp.child)))
        return sorted(edges, key=lambda edge: (by_name[edge[0]].id, by_name[edge[1]].id))

    def replace_keypoint(
        self,
        keypoint_id: int,
        *,
        link: str | None = None,
        local_xyz: tuple[float, float, float] | None = None,
        annotation_status: str = "manually_adjusted",
        note: str | None = None,
    ) -> None:
        updated = []
        found = False
        for kp in self.keypoints:
            if kp.id == keypoint_id:
                found = True
                updated.append(
                    dataclasses.replace(
                        kp,
                        link=link if link is not None else kp.link,
                        local_xyz=local_xyz if local_xyz is not None else kp.local_xyz,
                        annotation_status=annotation_status,
                        note=note if note is not None else kp.note,
                    )
                )
            else:
                updated.append(kp)
        if not found:
            raise KeyError(f"keypoint id {keypoint_id} not found")
        self.keypoints = updated
