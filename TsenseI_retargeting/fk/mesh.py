from __future__ import annotations

from pathlib import Path
import struct

import numpy as np


def stl_bounds(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    p = Path(path)
    data = p.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{p} is too small to be an STL file")
    if _looks_ascii_stl(data):
        return _ascii_stl_bounds(data.decode("utf-8", errors="ignore"))
    return _binary_stl_bounds(data, p)


def _looks_ascii_stl(data: bytes) -> bool:
    prefix = data[:80].lstrip().lower()
    return prefix.startswith(b"solid") and b"facet" in data[:512].lower()


def _binary_stl_bounds(data: bytes, path: Path) -> tuple[np.ndarray, np.ndarray]:
    triangles = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + triangles * 50
    if len(data) < expected:
        raise ValueError(f"{path} binary STL is truncated")
    mins = np.array([float("inf"), float("inf"), float("inf")], dtype=float)
    maxs = np.array([float("-inf"), float("-inf"), float("-inf")], dtype=float)
    offset = 84
    for _ in range(triangles):
        values = struct.unpack_from("<12f", data, offset)
        vertices = np.array(values[3:12], dtype=float).reshape(3, 3)
        mins = np.minimum(mins, vertices.min(axis=0))
        maxs = np.maximum(maxs, vertices.max(axis=0))
        offset += 50
    return mins, maxs


def _ascii_stl_bounds(text: str) -> tuple[np.ndarray, np.ndarray]:
    points = []
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            points.append([float(parts[1]), float(parts[2]), float(parts[3])])
    if not points:
        raise ValueError("ASCII STL contains no vertices")
    arr = np.asarray(points, dtype=float)
    return arr.min(axis=0), arr.max(axis=0)
