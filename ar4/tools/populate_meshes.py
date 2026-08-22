"""Populate models/meshes/ from the ar4_ros_driver checkout.

Annin ships the MK5 meshes as ASCII STL. MuJoCo's decoder only reads binary
STL, so this converts on the way across. models/meshes/ is gitignored -- it is
~49 MB of generated data with an upstream source -- so re-run this after a
fresh clone.

    python tools/populate_meshes.py
"""
import io
import re
import struct
import sys
from pathlib import Path

import numpy as np

# parents[1] is ar4/, parents[2] the repo root, parents[3] the directory the
# reference checkouts sit in alongside the repo.
_MESHES = (Path(__file__).resolve().parents[3] / "ar4_ros_driver"
           / "annin_ar4_description" / "meshes")
# The MK5 arm set and the SG1 gripper set ship in separate directories
# and disagree on filename case (*.STL vs *.stl), so match on suffix.
SRC_DIRS = (_MESHES / "ar4_mk5", _MESHES / "ar4_gripper")
DST = Path(__file__).resolve().parents[1] / "models" / "meshes"

MUJOCO_MAX_FACES = 200_000

_FACET = re.compile(rb"facet\s+normal\s+(\S+)\s+(\S+)\s+(\S+)")
_VERTEX = re.compile(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)")


def to_binary_stl(raw):
    """ASCII STL bytes -> binary STL bytes. Returns None if already binary."""
    if not raw.lstrip()[:5].lower().startswith(b"solid"):
        return None
    # A binary STL can also open with "solid", so confirm by looking for the
    # ASCII body keywords rather than trusting the header.
    if b"facet normal" not in raw[:4096] and b"facet normal" not in raw:
        return None

    normals = np.array(_FACET.findall(raw), dtype=np.float32)
    verts = np.array(_VERTEX.findall(raw), dtype=np.float32)
    n = len(normals)
    if n == 0 or len(verts) != 3 * n:
        raise ValueError(f"parsed {n} facets but {len(verts)} vertices")

    rec = np.zeros(n, dtype=np.dtype([("n", "<f4", 3),
                                      ("v", "<f4", (3, 3)),
                                      ("attr", "<u2")]))
    assert rec.itemsize == 50, f"packed facet must be 50 bytes, got {rec.itemsize}"
    rec["n"] = normals
    rec["v"] = verts.reshape(n, 3, 3)

    return b"\0" * 80 + struct.pack("<I", n) + rec.tobytes(), n


def face_count(raw):
    """Facet count of a binary STL, or None if it does not look like one."""
    if len(raw) < 84:
        return None
    n = struct.unpack("<I", raw[80:84])[0]
    return n if len(raw) - 84 == n * 50 else None


def decimate(raw, name, limit=MUJOCO_MAX_FACES):
    """Reduce a binary STL below MuJoCo's face limit.

    Three of the MK5 parts are CAD exports tessellated far past anything the
    renderer needs -- Link_Base_Aluminum alone is 259k faces. These are
    visual-only geoms, so quadric decimation costs nothing that matters.
    """
    import trimesh

    mesh = trimesh.load(io.BytesIO(raw), file_type="stl")
    target = int(limit * 0.9)
    reduced = mesh.simplify_quadric_decimation(face_count=target)
    out = io.BytesIO()
    reduced.export(out, file_type="stl")
    print(f"  {name:<28} decimated {len(mesh.faces)} -> {len(reduced.faces)} faces")
    return out.getvalue()


def main():
    missing = [d for d in SRC_DIRS if not d.is_dir()]
    if missing:
        sys.exit(f"source meshes not found: {missing[0]}")
    DST.mkdir(parents=True, exist_ok=True)

    converted = copied = 0
    sources = [f for d in SRC_DIRS for f in sorted(d.iterdir())
               if f.suffix.lower() == ".stl"]
    for src in sources:
        raw = src.read_bytes()
        result = to_binary_stl(raw)
        if result is None:
            n = face_count(raw)
            if n is not None and n > MUJOCO_MAX_FACES:
                raw = decimate(raw, src.name)
            (DST / src.name).write_bytes(raw)
            copied += 1
            continue
        data, faces = result
        (DST / src.name).write_bytes(data)
        converted += 1
        print(f"  {src.name:<28} {faces:>7} faces  "
              f"{len(raw) / 1e6:.1f} MB -> {len(data) / 1e6:.1f} MB")

    print(f"\n{converted} converted, {copied} copied, into {DST}")


if __name__ == "__main__":
    main()
