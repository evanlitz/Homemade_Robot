from __future__ import annotations

from pathlib import Path

import yaml

from .schema import ArchitectureConfig

CONFIG_DIR = Path(__file__).parent


class Placeholder(float):
    """A float that carries a flag: this number was invented, not chosen.

    Written in YAML as `!placeholder 0.35`. Arithmetic works normally, so
    placeholders propagate silently into calculations -- which is the point.
    `ArchitectureConfig.placeholder_fields()` finds them again afterwards.
    """

    __slots__ = ()

    def __repr__(self) -> str:
        return f"<PLACEHOLDER {float(self)!r}>"


class _Loader(yaml.SafeLoader):
    pass


_Loader.add_constructor(
    "!placeholder",
    lambda loader, node: Placeholder(loader.construct_scalar(node)),
)


def load_architecture(name_or_path: str | Path) -> ArchitectureConfig:
    path = Path(name_or_path)
    if not path.suffix:
        path = CONFIG_DIR / f"{path}.yaml"
    with path.open("r", encoding="utf-8") as fh:
        raw = yaml.load(fh, Loader=_Loader)
    return ArchitectureConfig.from_dict(raw, source=path)


def available_architectures() -> list[str]:
    return sorted(p.stem for p in CONFIG_DIR.glob("*.yaml"))
