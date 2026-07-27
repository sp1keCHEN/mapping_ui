"""Display helpers for occupancy-map PNG files."""

from __future__ import annotations

from pathlib import Path

from PIL import Image


TRANSPARENT_AREA_COLOR = (75, 99, 130, 255)


def occupancy_preview(path: Path) -> Image.Image:
    """Render transparent PNG pixels with a clearly distinct background color."""
    with Image.open(path) as source:
        image = source.convert("RGBA")
    background = Image.new("RGBA", image.size, TRANSPARENT_AREA_COLOR)
    return Image.alpha_composite(background, image).convert("RGB")
