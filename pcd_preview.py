"""Small dependency-free reader and orthographic previews for binary PCD files."""

from __future__ import annotations

from pathlib import Path
import struct

import numpy as np


_TYPE_CODES = {
    ("F", 4): "<f4", ("F", 8): "<f8",
    ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8",
    ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
}


def _lzf_decompress(data: bytes, expected_size: int) -> bytes:
    """Decompress the LZF payload used by PCL DATA binary_compressed."""
    output = bytearray(expected_size)
    ip = op = 0
    while ip < len(data):
        control = data[ip]
        ip += 1
        if control < 32:
            length = control + 1
            if ip + length > len(data) or op + length > expected_size:
                raise ValueError("invalid LZF literal block")
            output[op:op + length] = data[ip:ip + length]
            ip += length
            op += length
            continue
        length_code = control >> 5
        length = length_code + 2
        reference = op - 1 - ((control & 0x1F) << 8)
        if length_code == 7:
            if ip >= len(data):
                raise ValueError("invalid LZF back-reference length")
            length += data[ip]
            ip += 1
        if ip >= len(data):
            raise ValueError("invalid LZF back-reference")
        reference -= data[ip]
        ip += 1
        if reference < 0 or op + length > expected_size:
            raise ValueError("invalid LZF back-reference range")
        for index in range(length):
            output[op] = output[reference + index]
            op += 1
    if op != expected_size:
        raise ValueError("LZF payload size does not match PCD header")
    return bytes(output)


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
    data_format = header.get("DATA", [""])[0].lower()
    if data_format == "binary":
        if len(payload) < points * dtype.itemsize:
            raise ValueError("PCD binary payload is shorter than its header declares")
        cloud = np.frombuffer(payload, dtype=dtype, count=points)
    elif data_format == "binary_compressed":
        if len(payload) < 8:
            raise ValueError("compressed PCD payload is missing size fields")
        compressed_size, uncompressed_size = struct.unpack_from("<II", payload)
        compressed = payload[8:]
        if len(compressed) != compressed_size:
            raise ValueError("compressed PCD payload size does not match its header")
        raw = _lzf_decompress(compressed, uncompressed_size)
        expected_size = points * dtype.itemsize
        if uncompressed_size != expected_size:
            raise ValueError("compressed PCD uncompressed size does not match its fields")
        cloud = np.empty(points, dtype=dtype)
        cursor = 0
        for field, size, point_type, count in zip(fields, sizes, types, counts):
            field_dtype = np.dtype(_TYPE_CODES[(point_type.upper(), size)])
            field_bytes = points * size * count
            values = np.frombuffer(raw[cursor:cursor + field_bytes], dtype=field_dtype)
            if count > 1:
                values = values.reshape(points, count)
            cloud[field] = values
            cursor += field_bytes
    else:
        raise ValueError(f"unsupported PCD DATA format: {data_format}; only binary and binary_compressed are supported")
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


def projection_image(
    xyz: np.ndarray,
    horizontal_axis: int,
    vertical_axis: int,
    width: int = 720,
    height: int = 480,
) -> np.ndarray:
    """Rasterize an orthographic point-density preview for two XYZ axes."""
    if horizontal_axis == vertical_axis or {horizontal_axis, vertical_axis} - {0, 1, 2}:
        raise ValueError("projection axes must be two distinct XYZ indices")
    if len(xyz) == 0:
        raise ValueError("PCD contains no finite points")
    horizontal = xyz[:, horizontal_axis]
    vertical = xyz[:, vertical_axis]
    horizontal_span = max(float(horizontal.max() - horizontal.min()), 1e-3)
    vertical_span = max(float(vertical.max() - vertical.min()), 1e-3)
    px = np.clip(((horizontal - horizontal.min()) / horizontal_span * (width - 1)).astype(int), 0, width - 1)
    py = np.clip(((vertical - vertical.min()) / vertical_span * (height - 1)).astype(int), 0, height - 1)
    density = np.zeros((height, width), dtype=np.uint16)
    np.add.at(density, (height - 1 - py, px), 1)
    image = np.full((height, width, 3), 18, dtype=np.uint8)
    strength = np.clip(np.log1p(density) / max(np.log1p(int(density.max())), 1.0), 0, 1)
    image[..., 0] = (25 + strength * 35).astype(np.uint8)
    image[..., 1] = (35 + strength * 185).astype(np.uint8)
    image[..., 2] = (45 + strength * 210).astype(np.uint8)
    return image


def topdown_image(xyz: np.ndarray, width: int = 900, height: int = 600) -> np.ndarray:
    """Backward-compatible XY top-down preview."""
    return projection_image(xyz, 0, 1, width=width, height=height)


def three_view_images(xyz: np.ndarray) -> dict[str, np.ndarray]:
    """Return XY top, XZ front, and YZ side orthographic PCD previews."""
    return {
        "xy": projection_image(xyz, 0, 1),
        "xz": projection_image(xyz, 0, 2),
        "yz": projection_image(xyz, 1, 2),
    }
