import tempfile
import unittest
from pathlib import Path

from multimap import (
    RELATIONS_HEADER,
    TRANSITIONS_HEADER,
    deploy_project,
    inspect_multimap_dir,
    next_map_id,
    next_map_id_after,
    switch_map_request,
)


def write_map_bundle(root: Path) -> None:
    (root / "states").mkdir(parents=True)
    for map_id in ("map_000", "map_001"):
        (root / f"{map_id}.png").write_bytes(b"png")
        (root / f"{map_id}.yaml").write_text(f"image: {map_id}.png\n")
        (root / "states" / f"{map_id}.gridmap.bin").write_bytes(b"state")
    (root / "map_relations.csv").write_text(
        ",".join(RELATIONS_HEADER) + "\nROOT,map_000,0,0\nmap_000,map_001,1,2\n"
    )
    (root / "transition_points.csv").write_text(
        ",".join(TRANSITIONS_HEADER) + "\ntp_1,map_000,map_001,1,2,0,0,true,stairs\n"
    )


class MultiMapTests(unittest.TestCase):
    def test_next_map_id_and_switch_request(self):
        self.assertEqual(next_map_id(["map_000", "map_002"]), "map_001")
        self.assertEqual(next_map_id_after("map_000"), "map_001")
        self.assertEqual(next_map_id_after("map_003"), "map_004")
        self.assertEqual(next_map_id_after("floor_9"), "floor_10")
        self.assertEqual(switch_map_request("map_001", "stairs", True)["bidirectional"], 1)
        self.assertEqual(switch_map_request("map_001", "stairs", False)["bidirectional"], 2)

    def test_inspect_and_atomically_deploy_project(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            tmp_path = Path(temp_dir)
            output = tmp_path / "output"
            write_map_bundle(output)
            self.assertTrue(inspect_multimap_dir(output).valid)
            prior = tmp_path / "prior"
            (prior / "keyframes").mkdir(parents=True)
            (prior / "PGO.pcd").write_bytes(b"pcd")
            (prior / "keyframes" / "poses.txt").write_text("poses")
            target = deploy_project(prior, output, tmp_path / "Maps", "company2")
            self.assertTrue((target / "PGO.pcd").is_file())
            self.assertTrue((target / "map_000.yaml").is_file())
            self.assertTrue(inspect_multimap_dir(target).valid)

    def test_live_active_map_can_be_referenced_before_export(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            write_map_bundle(root)
            (root / "map_001.png").unlink()
            (root / "map_001.yaml").unlink()
            (root / "states" / "map_001.gridmap.bin").unlink()
            report = inspect_multimap_dir(root, allowed_unexported_map_ids={"map_001"})
            self.assertTrue(report.valid, report.errors)
