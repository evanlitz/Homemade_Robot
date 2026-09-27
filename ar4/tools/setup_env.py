"""Make a fresh clone ready to run everything, including the simulator tests.

    python tools/setup_env.py            # from ar4/, once after cloning

Two reference checkouts live BESIDE the repo, not in it, because neither is
ours to redistribute and one is large:

    <parent>/ar4_ros_driver   Annin-Robotics' fork: MK5 + SG1 meshes, and the
                              Teensy firmware backends/hw.py speaks
    <parent>/ar4-hmi          Annin's HMI: the compiled ARrobots kinematics
                              that tests/test_kinematics.py checks FK against
                              (non-commercial licence; used as a test oracle)

This clones each one at a pinned commit if it is not already there -- an
existing checkout is never touched -- then generates models/meshes/. Without
it, 13 tests skip: 12 need the meshes, 1 needs the oracle.

Pinned rather than tracking main so a change upstream cannot silently change
what the tests compare against. Bump a pin deliberately, then rerun the tests.
"""
import argparse
import subprocess
import sys
from pathlib import Path

AR4 = Path(__file__).resolve().parents[1]
# The directory the repo itself sits in. Fixed, not an option: conftest.py and
# populate_meshes.py both look for the checkouts exactly here.
PARENT = AR4.parents[1]

REFS = {
    "ar4_ros_driver": ("https://github.com/Annin-Robotics/ar4_ros_driver.git",
                       "6a3ebb11cedab12cd1d29b41d63aa270a008ab8b"),
    "ar4-hmi": ("https://github.com/Annin-Robotics/ar4-hmi.git",
                "ed2abfd3a33340745dd4e2e114a7b1ac6761fc29"),
}


def git(*args, cwd=None):
    subprocess.run(["git", *args], cwd=cwd, check=True)


def fetch(name, url, commit, parent):
    dest = parent / name
    if dest.exists():
        print(f"{name}: already at {dest}, leaving it alone")
        return dest
    print(f"{name}: cloning {url} @ {commit[:10]}")
    dest.mkdir(parents=True)
    # init + fetch of one commit: a full clone of ar4-hmi carries every
    # platform's compiled binaries through its whole history
    git("init", "-q", cwd=dest)
    git("remote", "add", "origin", url, cwd=dest)
    git("fetch", "-q", "--depth", "1", "origin", commit, cwd=dest)
    git("checkout", "-q", "FETCH_HEAD", cwd=dest)
    return dest


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--skip-meshes", action="store_true")
    args = ap.parse_args(argv)

    for name, (url, commit) in REFS.items():
        fetch(name, url, commit, PARENT)

    if not args.skip_meshes:
        sys.path.insert(0, str(AR4 / "tools"))
        import populate_meshes
        populate_meshes.main()

    tag = f"cpython-{sys.version_info.major}{sys.version_info.minor}"
    hmi = PARENT / "ar4-hmi" / "ARrobots"
    if hmi.is_dir() and not any(tag in p.name or p.suffix == ".pyd"
                                for p in hmi.glob("robot_kinematics*")):
        print(f"note: ar4-hmi ships no robot_kinematics build for {tag}; the FK "
              f"oracle test will skip. Python 3.11 or 3.12 has one.")
    print("\nready: python -m pytest -q")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
