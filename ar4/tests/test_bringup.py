"""tools/bringup.py, rehearsed against the fake Teensy with scripted answers.

What matters is that each failure the session exists to catch -- a wrong
offset, a reversed joint, an operator who says stop -- ends the session before
the next motion, and names the edit that fixes it.
"""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from backends.hw import required_slowdown
from motion.trajectory import Trajectory

TOOLS = Path(__file__).resolve().parents[1] / "tools"
spec = importlib.util.spec_from_file_location("bringup", TOOLS / "bringup.py")
bringup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bringup)


def fit(traj):
    return Trajectory(traj.q, traj.dt * required_slowdown(traj.q, traj.dt))


def answers(no_on=(), quit_on=None):
    """'n' to any confirm prompt containing a phrase in no_on, 'q' at the first
    prompt containing quit_on, 'y' to other confirms, Enter otherwise."""
    asked = []

    def ask(prompt):
        asked.append(prompt)
        if quit_on and quit_on in prompt:
            return "q"
        if "[y/n/q]" in prompt:
            return "n" if any(p in prompt for p in no_on) else "y"
        return ""
    ask.asked = asked
    return ask


def run(tmp_path, arm=None, gripper=True, **kw):
    arm = arm or bringup.rehearsal_arm(gripper=gripper)
    sent = arm._io.sent
    session = bringup.Bringup(arm, ask=kw.pop("ask", answers()), log=lambda *_: None,
                              gains=(2.0, 4.0), out_dir=tmp_path)
    return session.run(fit, gripper=gripper), sent, arm


def motion_after(sent, marker):
    """Motion commands sent after the first line starting with `marker`."""
    idx = max(i for i, s in enumerate(sent) if s.startswith(marker))
    return [s for s in sent[idx + 1:] if s.startswith(("MT", "MV"))]


def test_clean_session_passes_and_writes_a_report(tmp_path):
    report, _, arm = run(tmp_path)
    assert report["result"] == "passed", report["result"]
    assert report["code_changes"] == []
    assert set(report["steps"]) == {"connect", "home", "park", "direction",
                                    "tracking", "gripper"}
    assert all(r["confirmed"] for r in report["steps"]["direction"].values())
    lag = report["steps"]["tracking"]["worst_tip_mm_by_gain"]
    assert set(lag) == {"2.0", "4.0"} and all(v < 5 for v in lag.values())
    assert (tmp_path / "bringup-tracking.csv").exists()
    saved = json.loads(next(tmp_path.glob("bringup-*.json")).read_text())
    assert saved["result"] == "passed"
    assert not arm.connected


def test_wrong_offset_is_caught_at_park_before_anything_else_moves(tmp_path):
    arm = bringup.rehearsal_arm()
    arm.offsets = arm.offsets.copy()
    arm.offsets[2] = -80.0            # true MK5 value is -89
    report, sent, _ = run(tmp_path, arm=arm)
    assert report["result"].startswith("stopped")
    assert "park" in report["result"]
    (change,) = report["code_changes"]
    assert "J3" in change and "-89.00" in change
    assert motion_after(sent, "JC") == []


def test_reversed_joint_names_the_sign_to_flip_and_stops(tmp_path):
    report, sent, _ = run(tmp_path, ask=answers(no_on=("did J2",)))
    assert report["result"].startswith("stopped")
    assert any("JOINT_SIGNS[1]" in c for c in report["code_changes"])
    assert "tracking" not in report["steps"]


def test_q_at_the_first_prompt_moves_nothing(tmp_path):
    report, sent, arm = run(tmp_path, ask=answers(quit_on="home"))
    assert report["result"] == "stopped: stopped by operator"
    assert not any(s.startswith(("JC", "MT", "MV")) for s in sent)
    assert not arm.connected


def test_no_to_clearance_moves_nothing(tmp_path):
    report, sent, _ = run(tmp_path, ask=answers(no_on=("Arm clear",)))
    assert report["result"].startswith("stopped")
    assert not any(s.startswith(("JC", "MT", "MV")) for s in sent)


def test_predictions_match_the_conventions():
    park = np.zeros(6)
    j1 = bringup.describe_jog(park, 0)
    assert "counterclockwise" in j1 and "left" in j1
    assert "down" in bringup.describe_jog(park, 1)
    wrist = park.copy()
    wrist[4] = 30.0
    assert "wrist" in bringup.describe_jog(wrist, 3)
    assert "flange" in bringup.describe_jog(wrist, 5)


def test_main_needs_a_port_or_rehearse():
    with pytest.raises(SystemExit):
        bringup.main([])
