"""Pure helpers for GridMapper multi-map mapping projects.

These functions intentionally do not import Streamlit or ROS2 so they can be
validated on a development PC without lidar hardware.
"""

from __future__ import annotations

import csv
import os
import re
import shutil
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


MAP_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,47}$")
RELATIONS_HEADER = ["from_map", "to_map", "dx", "dy"]
TRANSITIONS_HEADER = [
    "transition_id", "from_map", "to_map", "world_x", "world_y",
    "world_z", "world_yaw_rad", "bidirectional", "type",
]
TRANSITION_TYPES = ("stairs", "door", "elevator", "corridor", "manual")


@dataclass(frozen=True)
class MultiMapReport:
    map_ids: list[str]
    errors: list[str]
    relations_count: int
    transitions_count: int

    @property
    def valid(self) -> bool:
        return not self.errors


def is_valid_map_id(value: str) -> bool:
    return bool(MAP_ID_RE.fullmatch(value))


def next_map_id(map_ids: list[str]) -> str:
    """Return the first unused map_NNN ID, starting with map_000."""
    used = set(map_ids)
    index = 0
    while f"map_{index:03d}" in used:
        index += 1
    return f"map_{index:03d}"


def next_map_id_after(map_id: str) -> str:
    """Increment a numeric map suffix while preserving its prefix and width."""
    match = re.fullmatch(r"(.*?)(\d+)", map_id)
    if not match:
        return next_map_id([map_id])
    prefix, number = match.groups()
    return f"{prefix}{int(number) + 1:0{len(number)}d}"


def switch_map_request(target_map: str, transition_type: str, bidirectional: bool) -> dict[str, object]:
    if not is_valid_map_id(target_map):
        raise ValueError(f"invalid map id: {target_map!r}")
    if transition_type not in TRANSITION_TYPES:
        raise ValueError(f"unsupported transition type: {transition_type!r}")
    return {
        "target_map": target_map,
        "transition_type": transition_type,
        "bidirectional": 1 if bidirectional else 2,
        "create_if_missing": 1,
        "switch_active_map": 1,
    }


def _read_csv(path: Path, expected_header: list[str], errors: list[str]) -> list[list[str]]:
    if not path.is_file():
        errors.append(f"missing {path.name}")
        return []
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.reader(handle))
    except OSError as exc:
        errors.append(f"cannot read {path.name}: {exc}")
        return []
    if not rows or rows[0] != expected_header:
        errors.append(f"invalid header in {path.name}")
        return []
    return [row for row in rows[1:] if any(cell.strip() for cell in row)]


def inspect_multimap_dir(
    directory: Path, allowed_unexported_map_ids: set[str] | None = None,
) -> MultiMapReport:
    """Validate the portable GridMapper output consumed by multi_map_nav."""
    errors: list[str] = []
    if not directory.is_dir():
        return MultiMapReport([], [f"missing multi-map directory: {directory}"], 0, 0)

    map_ids = sorted(path.stem for path in directory.glob("*.yaml") if is_valid_map_id(path.stem))
    if not map_ids:
        errors.append("no map YAML files found")
    for map_id in map_ids:
        if not (directory / f"{map_id}.png").is_file():
            errors.append(f"missing {map_id}.png")
        states_dir = directory / "states"
        compressed_states = list(states_dir.glob(f"{map_id}_*x*c.gridmap.bin.gz"))
        compressed_state = states_dir / f"{map_id}.gridmap.bin.gz"
        legacy_state = states_dir / f"{map_id}.gridmap.bin"
        if not (compressed_states or compressed_state.is_file() or legacy_state.is_file()):
            errors.append(f"missing states/{map_id}.gridmap.bin.gz (or legacy .bin)")

    relations = _read_csv(directory / "map_relations.csv", RELATIONS_HEADER, errors)
    transitions = _read_csv(directory / "transition_points.csv", TRANSITIONS_HEADER, errors)
    # During online mapping GridMapper persists relation/transition rows as
    # soon as it switches maps, while the new active map PNG/YAML is exported
    # only when that map is saved. The current active ID is a valid temporary
    # reference even before its files are written.
    known = set(map_ids) | (allowed_unexported_map_ids or set())
    if relations and not any(len(row) == 4 and row[0] == "ROOT" for row in relations):
        errors.append("map_relations.csv has no ROOT relation")
    for row in relations:
        if len(row) != 4:
            errors.append("map_relations.csv has a malformed row")
        elif (row[0] != "ROOT" and row[0] not in known) or row[1] not in known:
            errors.append("map_relations.csv references an unknown map")
    for row in transitions:
        if len(row) != 9:
            errors.append("transition_points.csv has a malformed row")
        elif row[1] not in known or row[2] not in known:
            errors.append("transition_points.csv references an unknown map")
    return MultiMapReport(map_ids, errors, len(relations), len(transitions))


def read_multimap_tables(directory: Path) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    """Read display-only relation and transition rows from a validated output."""
    tables: list[list[dict[str, str]]] = []
    for filename, header in (("map_relations.csv", RELATIONS_HEADER), ("transition_points.csv", TRANSITIONS_HEADER)):
        path = directory / filename
        try:
            with path.open(newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                if reader.fieldnames != header:
                    return [], []
                tables.append([dict(row) for row in reader if any((value or "").strip() for value in row.values())])
        except OSError:
            return [], []
    return tables[0], tables[1]


def archive_previous_output(output_root: Path) -> Path | None:
    """Archive a previous multi_maps directory before a new project starts."""
    source = output_root / "multi_maps"
    if not source.exists():
        return None
    archive = output_root / f"multi_maps_archive_{datetime.now():%Y%m%d_%H%M%S}"
    suffix = 1
    while archive.exists():
        archive = output_root / f"multi_maps_archive_{datetime.now():%Y%m%d_%H%M%S}_{suffix}"
        suffix += 1
    source.rename(archive)
    return archive


def deploy_project(prior_dir: Path, multi_map_dir: Path, maps_root: Path, project_name: str) -> Path:
    """Atomically publish a prior + GridMapper result as one portable project."""
    if not is_valid_map_id(project_name):
        raise ValueError(f"invalid project name: {project_name!r}")
    if not (prior_dir / "PGO.pcd").is_file() or not (prior_dir / "keyframes").is_dir():
        raise ValueError(f"prior is incomplete: {prior_dir}")
    report = inspect_multimap_dir(multi_map_dir)
    if not report.valid:
        raise ValueError("multi-map output is incomplete: " + "; ".join(report.errors))

    maps_root.mkdir(parents=True, exist_ok=True)
    target = maps_root / project_name
    staging = maps_root / f".{project_name}.staging"
    backup = maps_root / f".{project_name}.backup"
    shutil.rmtree(staging, ignore_errors=True)
    shutil.rmtree(backup, ignore_errors=True)
    staging.mkdir()
    try:
        shutil.copy2(prior_dir / "PGO.pcd", staging / "PGO.pcd")
        shutil.copytree(prior_dir / "keyframes", staging / "keyframes")
        for entry in multi_map_dir.iterdir():
            destination = staging / entry.name
            if entry.is_dir():
                shutil.copytree(entry, destination)
            else:
                shutil.copy2(entry, destination)
        published = inspect_multimap_dir(staging)
        if not published.valid:
            raise ValueError("staging validation failed: " + "; ".join(published.errors))
        if target.exists():
            os.replace(target, backup)
        os.replace(staging, target)
        shutil.rmtree(backup, ignore_errors=True)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        if backup.exists() and not target.exists():
            os.replace(backup, target)
        raise
    return target
