"""In-tree P5/P2 PGM parser (no ROS) for the GUI map canvas. PLAN.md T4.1.

Qt's PNM image plugin isn't guaranteed in a minimal PyQt5, so the operator GUI parses map.pgm here
and builds a Format_Grayscale8 QImage. Minimal real implementation of the header parse + P5 (binary)
and P2 (ascii) bodies; full edge-case tests in PLAN.md T4.1.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Pgm:
    width: int
    height: int
    maxval: int
    pixels: bytes  # row-major, one byte per pixel (0..255)


def _tokens(data: bytes):
    """Yield whitespace-separated tokens, skipping # comments to end-of-line."""
    i, n = 0, len(data)
    while i < n:
        c = data[i:i + 1]
        if c in b' \t\r\n':
            i += 1
        elif c == b'#':
            while i < n and data[i:i + 1] != b'\n':
                i += 1
        else:
            j = i
            while j < n and data[j:j + 1] not in b' \t\r\n#':
                j += 1
            yield data[i:j]
            i = j


def parse(data: bytes) -> Pgm:
    """Parse P2 (ascii) or P5 (binary) PGM bytes into a Pgm (8-bit grayscale)."""
    if data[:2] not in (b'P2', b'P5'):
        raise ValueError("not a P2/P5 PGM")
    magic = data[:2]
    it = _tokens(data[2:])
    width = int(next(it)); height = int(next(it)); maxval = int(next(it))
    if magic == b'P2':
        vals = bytes(min(255, int(t) * 255 // maxval) for t in it)
        if len(vals) < width * height:
            raise ValueError("truncated P2 body")
        return Pgm(width, height, 255, vals[:width * height])
    # P5 (binary): the body is `width*height` raw bytes beginning immediately after the single
    # whitespace char that follows maxval. Locating that offset robustly (comments may precede it)
    # plus maxval!=255 scaling and edge-case tests are finished in PLAN.md T4.1.
    raise NotImplementedError("PLAN.md T4.1 — finish P5 binary-body offset + scaling + tests")
