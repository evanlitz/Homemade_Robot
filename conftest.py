"""Resolve imports relative to this file, not the shell's working directory.

Puts the package root on sys.path so `motion` / `backends` import, and the
sibling HMI checkout on sys.path so the `ARrobots` oracle imports. pytest loads
this before collection, so both work regardless of where pytest is invoked from.
"""
import sys
from pathlib import Path

_ROOT = Path(__file__).parent
sys.path[:0] = [str(_ROOT), str(_ROOT.parent / "ar4-hmi")]
