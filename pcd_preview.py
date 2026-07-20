"""Small dependency-free reader and top-down preview for binary PCD files."""

from __future__ import annotations

from pathlib import Path

import numpy as np


_TYPE_CODES = {
    ("F", 4): "<f4", ("F", 8): "<f8",
    ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8",
    ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
}


def load_xyz(path: Path, max_points: int = 25_000) -> np.ndarray:
    """Load sampled XYZ values from a PCD v0.7 binary file."""
    with path.open("rb") as handle:
        header_lines: list[str] = []
        while True:
            raw = handle.readline()
            if not raw:
                raise ValueError("PCD header is incomplete")
            line = raw.decode("ascii", errors="strict").strip()
            header_lines.append(line)
            if line.upper().startswith("DATA "):
                break
        offset = handle.tell()

    header: dict[str, list[str]] = {}
    for line in header_lines:
        parts = line.split()
        if len(parts) >= 2:
            header[parts[0].upper()] = parts[1:]
    if header.get("DATA", [""])[0].lower() != "binary":
        raise ValueError("only binary PCD is supported for browser preview")
    fields = header.get("FIELDS", [])
    sizes = [int(value) for value in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    points = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
    if not fields or len(fields) != len(sizes) or len(fields) != len(types) or len(fields) != len(counts):
        raise ValueError("PCD field metadata is invalid")
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PCD does not contain x/y/z fields")

    dtype_fields = []
    for field, size, point_type, count in zip(fields, sizes, types, counts):
        code = _TYPE_CODES.get((point_type.upper(), size))
        if code is None:
            raise ValueError(f"unsupported PCD field type: {point_type}{size}")
        dtype_fields.append((field, code, (count,)) if count > 1 else (field, code))
    dtype = np.dtype(dtype_fields)
    payload = path.read_bytes()[offset:]
    if len(payload) < points * dtype.itemsize:
        raise ValueError("PCD binary payload is shorter than its header declares")
    cloud = np.frombuffer(payload, dtype=dtype, count=points)
    xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(np.float32, copy=False)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) > max_points:
        xyz = xyz[np.linspace(0, len(xyz) - 1, max_points, dtype=np.int64)]
    return xyz


def preview_summary(xyz: np.ndarray) -> dict[str, float | int]:
    if len(xyz) == 0:
        raise ValueError("PCD contains no finite points")
    minimum = xyz.min(axis=0)
    maximum = xyz.max(axis=0)
    return {
        "points": int(len(xyz)),
        "x_min": float(minimum[0]), "x_max": float(maximum[0]),
        "y_min": float(minimum[1]), "y_max": float(maximum[1]),
        "z_min": float(minimum[2]), "z_max": float(maximum[2]),
    }


def topdown_image(xyz: np.ndarray, width: int = 900, height: int = 600) -> np.ndarray:
    """Rasterize an XY top-down point-density preview without plotting packages."""
    summary = preview_summary(xyz)
    x_span = max(summary["x_max"] - summary["x_min"], 1e-3)
    y_span = max(summary["y_max"] - summary["y_min"], 1e-3)
    px = np.clip(((xyz[:, 0] - summary["x_min"]) / x_span * (width - 1)).astype(int), 0, width - 1)
    py = np.clip(((xyz[:, 1] - summary["y_min"]) / y_span * (height - 1)).astype(int), 0, height - 1)
    density = np.zeros((height, width), dtype=np.uint16)
    np.add.at(density, (height - 1 - py, px), 1)
    image = np.full((height, width, 3), 18, dtype=np.uint8)
    strength = np.clip(np.log1p(density) / max(np.log1p(int(density.max())), 1.0), 0, 1)
    image[..., 0] = (25 + strength * 35).astype(np.uint8)
    image[..., 1] = (35 + strength * 185).astype(np.uint8)
    image[..., 2] = (45 + strength * 210).astype(np.uint8)
    return image
