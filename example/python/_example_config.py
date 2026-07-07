"""Small helpers shared by hardware Python examples."""
from __future__ import annotations

import argparse
import atexit
import tempfile
from pathlib import Path


_ROOT = Path(__file__).resolve().parents[2]
_CONFIG_DIR = _ROOT / "python" / "rebotarm_control_rt" / "config"


def add_port_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--port",
        default=None,
        help="Override the YAML channel, for example /dev/ttyACM0, /dev/ttyACM1, or can0.",
    )


def default_config_path(*, gripper: bool = False) -> Path:
    return _CONFIG_DIR / ("gripper.yaml" if gripper else "arm.yaml")


def _normalize_config_arg(config: str | None) -> str | None:
    if config is None:
        return None
    if str(config).strip() == "":
        raise ValueError(
            "--config was provided but is empty. This usually means a shell variable expanded "
            "to an empty string. Use the explicit path instead, for example: "
            "--config python/rebotarm_control_rt/config/arm_rs.yaml"
        )
    return config


def config_with_port(config: str | None, port: str | None, *, gripper: bool = False) -> str | None:
    config = _normalize_config_arg(config)
    if not port:
        return config

    src = Path(config) if config else default_config_path(gripper=gripper)
    text = src.read_text(encoding="utf-8")
    lines = []
    replaced = False

    for line in text.splitlines():
        stripped = line.lstrip()
        indent = line[: len(line) - len(stripped)]
        if not replaced and stripped.startswith("channel:"):
            lines.append(f"{indent}channel: {port}")
            replaced = True
        else:
            lines.append(line)

    if not replaced:
        raise ValueError(f"{src} does not contain a channel: entry")

    tmp = tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        suffix=".yaml",
        prefix="rebotarm_rt_example_",
        delete=False,
    )
    with tmp:
        tmp.write("\n".join(lines))
        tmp.write("\n")
    atexit.register(lambda path=tmp.name: Path(path).unlink(missing_ok=True))
    return tmp.name


def _strip_yaml_scalar(value: str) -> str:
    value = value.split("#", 1)[0].strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        value = value[1:-1]
    return value


def _yaml_top_level_scalar(path: str | Path | None, key: str, *, gripper: bool = False) -> str | None:
    if isinstance(path, str):
        path = _normalize_config_arg(path)
    src = Path(path) if path else default_config_path(gripper=gripper)
    try:
        lines = src.read_text(encoding="utf-8").splitlines()
    except OSError:
        return None
    prefix = f"{key}:"
    for line in lines:
        if line[:1].isspace():
            continue
        stripped = line.strip()
        if stripped.startswith(prefix):
            value = _strip_yaml_scalar(stripped[len(prefix):])
            return value or None
    return None


def _resolve_repo_path(path: str | Path, *, base: Path | None = None) -> Path:
    p = Path(path).expanduser()
    if p.is_absolute():
        return p
    candidates = []
    if base is not None:
        candidates.append(base / p)
    candidates.extend([_ROOT / p, Path.cwd() / p, p])
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def config_urdf_path(config: str | None) -> Path | None:
    config = _normalize_config_arg(config)
    value = _yaml_top_level_scalar(config, "urdf_path")
    if value is None:
        return None
    src = Path(config) if config else default_config_path()
    return _resolve_repo_path(value, base=src.expanduser().parent)


def config_end_effector_frame(config: str | None) -> str | None:
    config = _normalize_config_arg(config)
    return _yaml_top_level_scalar(config, "end_effector_frame")


def model_urdf_for_config(config: str | None, explicit_urdf: str | Path | None = None) -> str | None:
    if explicit_urdf:
        return str(explicit_urdf)
    path = config_urdf_path(config)
    return None if path is None else str(path)
