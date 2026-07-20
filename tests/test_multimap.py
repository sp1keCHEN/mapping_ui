import struct
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
from pcd_preview import load_xyz, preview_summary, topdown_image


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


def write_binary_pcd(path: Path) -> None:
    points = [(1.0, 2.0, 3.0), (-4.0, 5.0, -6.0)]
    header = """# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n"""
    path.write_bytes(header.encode() + b"".join(struct.pack("<ffff", x, y, z, 1.0) for x, y, z in points))


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

    def test_binary_pcd_xyz_and_preview(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            pcd = Path(temp_dir) / "map.pcd"
            write_binary_pcd(pcd)
            xyz = load_xyz(pcd)
            self.assertEqual(xyz.shape, (2, 3))
            self.assertEqual(preview_summary(xyz)["points"], 2)
            self.assertEqual(topdown_image(xyz, width=32, height=16).shape, (16, 32, 3))

    def test_rejects_non_binary_pcd(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            pcd = Path(temp_dir) / "ascii.pcd"
            pcd.write_text("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 0\nDATA ascii\n")
            with self.assertRaisesRegex(ValueError, "binary"):
                load_xyz(pcd)
