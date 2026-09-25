"""app.pullout: the bench target is derived, and the verdict refuses to guess."""
import math

import numpy as np
import pytest

from app.pullout import (OperatingPoint, available_nm, load_curve, motor_rpm,
                         operating_points, verdict)
from config import load_architecture


def _curve(tmp_path, rows, name="c.csv"):
    p = tmp_path / name
    p.write_text("# bench notes\nrpm,torque_nm\n"
                 + "".join(f"{r},{t}\n" for r, t in rows))
    return load_curve(p)


def test_rpm_matches_the_hand_numbers():
    """D14: 2.23 rad/s at 1:25 is 533 rpm. F18: 0.46 m/s on a 32 mm lead is
    ~860 rpm. The tool must reproduce both from config, not restate them."""
    cfg = load_architecture("scara")
    by = {j.name: motor_rpm(j) for j in cfg.joints}
    assert by["q0_shoulder"] == pytest.approx(2.23 * 25 * 60 / (2 * math.pi))
    assert by["q2_lift"] == pytest.approx(0.46 / 0.032 * 60)


def test_scara_operating_points_agree_with_f19():
    points, n = operating_points("scara")
    assert n > 0
    rev = [p for p in points if not p.linear]
    # F19: 0.213 N.m motor-side at the worst revolute demand
    assert max(p.motor_torque_nm for p in rev) == pytest.approx(0.213, abs=0.01)


def test_template_has_no_data(tmp_path):
    from pathlib import Path
    tpl = Path(__file__).parents[1] / "bench" / "pullout_TEMPLATE.csv"
    with pytest.raises(ValueError, match="at least 3"):
        load_curve(tpl)


def test_repeat_runs_keep_the_worst(tmp_path):
    rpm, tq = _curve(tmp_path, [(0, 8), (500, 4.0), (500, 3.5), (1000, 2)])
    assert list(rpm) == [0, 500, 1000]
    assert tq[1] == 3.5


def test_no_extrapolation(tmp_path):
    c = _curve(tmp_path, [(0, 8), (300, 6), (600, 4)])
    assert available_nm(c, 450) == pytest.approx(5.0)
    assert available_nm(c, 862) is None


def test_verdict(tmp_path):
    c = _curve(tmp_path, [(0, 8), (500, 4), (1000, 1)])
    pts = [OperatingPoint("a", False, 0, 0.5, 500),    # 8x
           OperatingPoint("b", False, 0, 1.0, 900),    # 1.6 / 1 = 1.6x
           OperatingPoint("c", True, 0, 0.1, 1200)]    # beyond the curve
    got = {r["point"].joint: r for r in verdict(pts, c, margin=2.0)}
    assert got["a"]["status"] == "PASS"
    assert got["b"]["status"] == "FAIL" and got["b"]["ratio"] == pytest.approx(1.6)
    assert got["c"]["status"] == "NOT MEASURED"
