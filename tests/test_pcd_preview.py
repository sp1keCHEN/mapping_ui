import struct
import tempfile
import unittest
from pathlib import Path

from pcd_preview import load_xyz, preview_summary, topdown_image


def write_binary_pcd(path: Path) -> None:
    points = [(1.0, 2.0, 3.0), (-4.0, 5.0, -6.0)]
    header = """# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n"""
    path.write_bytes(header.encode() + b"".join(struct.pack("<ffff", x, y, z, 1.0) for x, y, z in points))


class PcdPreviewTests(unittest.TestCase):
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
