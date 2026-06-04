"""In-tree P5/P2 PGM parser (no ROS) for the GUI map canvas. Implemented TDD per PLAN T4.1.

Qt's PNM image plugin isn't guaranteed in a minimal PyQt5, so the operator GUI parses map.pgm here
and builds a Format_Grayscale8 QImage. Handles binary (P5) and ascii (P2) bodies, `#` comments in
the header, and maxval scaling to 8-bit. Tests: test/test_pgm.py.
"""
from __future__ import annotations

from dataclasses import dataclass

_WS = b' \t\r\n'


@dataclass(frozen=True)
class Pgm:
    width: int
    height: int
    maxval: int
    pixels: bytes  # row-major, one byte per pixel (0..255)


def _read_header(data: bytes) -> tuple[int, int, int, int]:
    """Parse the 3 header ints after the magic, skipping whitespace and `#` comments.

    Returns (width, height, maxval, body_start) where body_start is the offset of the first body
    byte — i.e. one past the single whitespace separator that terminates the maxval token (PNM spec).
    """
    n = len(data)
    pos = 2  # past the 2-byte magic

    def skip_ws_comments(p: int) -> int:
        while p < n:
            c = data[p:p + 1]
            if c in _WS:
                p += 1
            elif c == b'#':
                while p < n and data[p:p + 1] != b'\n':
                    p += 1
            else:
                break
        return p

    vals = []
    for _ in range(3):
        pos = skip_ws_comments(pos)
        start = pos
        while pos < n and data[pos:pos + 1] not in _WS and data[pos:pos + 1] != b'#':
            pos += 1
        if pos == start:
            raise ValueError("truncated PGM header")
        vals.append(int(data[start:pos]))
    # The body starts after the maxval token's single whitespace terminator (PNM spec) — but do
    # NOT hard-code pos+1: that dropped/shifted every pixel when the maxval line ended in CRLF
    # (body began on the leftover '\n') or when a '#' comment abutted maxval (GIMP/hand-edited
    # maps). Tolerate both: skip an abutting comment, then consume CRLF as one logical separator.
    p = pos
    if p < n and data[p:p + 1] == b'#':
        while p < n and data[p:p + 1] != b'\n':
            p += 1
    if data[p:p + 2] == b'\r\n':
        p += 2
    elif p < n and data[p:p + 1] in _WS:
        p += 1
    return vals[0], vals[1], vals[2], p


def parse(data: bytes) -> Pgm:
    """Parse P2 (ascii) or P5 (binary) PGM bytes into a Pgm (8-bit grayscale)."""
    magic = data[:2]
    if magic not in (b'P2', b'P5'):
        raise ValueError("not a P2/P5 PGM")
    width, height, maxval, body = _read_header(data)
    if width <= 0 or height <= 0 or maxval <= 0:
        raise ValueError("bad PGM header dimensions")
    if maxval > 255:
        raise ValueError("16-bit PGM not supported (maxval > 255)")
    count = width * height

    if magic == b'P5':
        raw = data[body:body + count]
        if len(raw) < count:
            raise ValueError("truncated P5 body")
        pixels = raw if maxval == 255 else bytes(min(255, b * 255 // maxval) for b in raw)
        return Pgm(width, height, 255, pixels)

    # P2 (ascii): the body is whitespace-separated integers.
    vals = bytes(min(255, int(t) * 255 // maxval) for t in _tokens(data[body:]))
    if len(vals) < count:
        raise ValueError("truncated P2 body")
    return Pgm(width, height, 255, vals[:count])


def _tokens(data: bytes):
    """Yield whitespace-separated tokens, skipping `#` comments to end-of-line (P2 body only)."""
    i, n = 0, len(data)
    while i < n:
        c = data[i:i + 1]
        if c in _WS:
            i += 1
        elif c == b'#':
            while i < n and data[i:i + 1] != b'\n':
                i += 1
        else:
            j = i
            while j < n and data[j:j + 1] not in _WS and data[j:j + 1] != b'#':
                j += 1
            yield data[i:j]
            i = j
