"""TDD for lib/pgm.py — in-tree P5/P2 PGM parser for the GUI map canvas (PLAN T4.1).

Covers binary (P5) + ascii (P2) bodies, maxval scaling, comments in the header, error paths, and a
real parse of the course base map maps/map.pgm so the GUI's canvas source is trustworthy.
"""
import os

import pytest

from algae_dt.lib import pgm

MAP_PGM = os.path.join(os.path.dirname(__file__), '..', 'maps', 'map.pgm')


def test_parse_p5_binary():
    data = b"P5\n2 2\n255\n" + bytes([0, 85, 170, 255])
    img = pgm.parse(data)
    assert (img.width, img.height, img.maxval) == (2, 2, 255)
    assert img.pixels == bytes([0, 85, 170, 255])


def test_parse_p2_ascii():
    data = b"P2\n2 2\n255\n0 85 170 255\n"
    img = pgm.parse(data)
    assert (img.width, img.height) == (2, 2)
    assert img.pixels == bytes([0, 85, 170, 255])


def test_parse_p5_scales_when_maxval_not_255():
    data = b"P5\n1 2\n100\n" + bytes([50, 100])
    img = pgm.parse(data)
    assert img.maxval == 255
    assert img.pixels == bytes([50 * 255 // 100, 255])   # -> [127, 255]


def test_parse_handles_comments_in_header():
    data = b"P5\n# the course base map\n2 2\n255\n" + bytes([1, 2, 3, 4])
    img = pgm.parse(data)
    assert (img.width, img.height) == (2, 2) and img.pixels == bytes([1, 2, 3, 4])


def test_parse_p5_ignores_trailing_bytes_beyond_body():
    data = b"P5\n2 1\n255\n" + bytes([10, 20, 99, 99])   # 2 valid + trailing
    img = pgm.parse(data)
    assert img.pixels == bytes([10, 20])


def test_parse_rejects_non_pgm():
    with pytest.raises(ValueError):
        pgm.parse(b"BM\x00\x00not a pgm")


def test_parse_rejects_truncated_p5():
    with pytest.raises(ValueError):
        pgm.parse(b"P5\n2 2\n255\n" + bytes([0, 85]))     # needs 4, has 2


def test_parse_rejects_16bit():
    with pytest.raises(ValueError):
        pgm.parse(b"P5\n1 1\n65535\n\x00\x10")


def test_parse_real_course_map():
    with open(MAP_PGM, 'rb') as f:
        img = pgm.parse(f.read())
    # twin.yaml + map.yaml: 86 x 110 @ 0.05 m/px, 8-bit.
    assert (img.width, img.height) == (86, 110)
    assert img.maxval == 255
    assert len(img.pixels) == 86 * 110


def test_parse_p5_crlf_header_does_not_shift_body():
    """A spec-valid PGM saved on Windows ends header lines in CRLF. Hard-coding body_start=pos+1
    landed the body on the leftover '\\n', shifting every pixel and dropping the last (e.g.
    [10,10,20,30] instead of [10,20,30,40]). The map source feeds the GUI canvas + occupancy."""
    data = b"P5\r\n2 2\r\n255\r\n" + bytes([10, 20, 30, 40])
    img = pgm.parse(data)
    assert img.pixels == bytes([10, 20, 30, 40])


def test_parse_p5_comment_after_maxval_is_not_read_as_pixels():
    """PNM allows a comment after any header token; GIMP/hand-edited maps place one right after
    maxval. The old offset read the comment text ' c\\n' + first byte as pixels."""
    data = b"P5\n2 2\n255# editor note\n" + bytes([1, 2, 3, 4])
    img = pgm.parse(data)
    assert img.pixels == bytes([1, 2, 3, 4])


def test_parse_p2_scales_when_maxval_not_255():
    """The P2 ascii maxval-scaling branch (only P5 scaling was covered)."""
    data = b"P2\n1 2\n100\n50 100\n"
    img = pgm.parse(data)
    assert img.maxval == 255
    assert img.pixels == bytes([50 * 255 // 100, 255])   # -> [127, 255]
