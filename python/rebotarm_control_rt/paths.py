"""Repository and resource paths used by examples and thin Python wrappers."""
from __future__ import annotations

from pathlib import Path


def package_root() -> Path:
    return Path(__file__).resolve().parent


def repo_root() -> Path:
    """Return the source repository root when running from a checkout.

    In an installed wheel there may be no repository root next to the package;
    callers should use the explicit resource paths below rather than assuming
    this path exists.
    """
    source_root = package_root().parents[1]
    if (source_root / "pyproject.toml").exists() and (source_root / "urdf").exists():
        return source_root

    cwd = Path.cwd()
    if (cwd / "pyproject.toml").exists() and (cwd / "urdf").exists():
        return cwd

    return source_root


def default_urdf_path() -> Path:
    """Return the default reBot-DevArm URDF path.

    Source checkouts keep URDF assets at the project level under ``urdf/``.
    The package-local fallback keeps older installs usable if they still bundle
    the URDF under ``python/rebotarm_control_rt/urdf``.
    """
    rel = Path("reBot-DevArm_fixend_description") / "urdf" / "reBot-DevArm_fixend.urdf"
    candidates = [
        repo_root() / "urdf" / rel,
        package_root().parent / "urdf" / rel,
        package_root().parents[2] / "urdf" / rel,
        Path.cwd() / "urdf" / rel,
        package_root() / "urdf" / rel,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def _strip_yaml_scalar(value: str) -> str:
    value = value.split("#", 1)[0].strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        value = value[1:-1]
    return value


def _yaml_top_level_scalar(path: str | Path, key: str) -> str | None:
    try:
        lines = Path(path).expanduser().read_text(encoding="utf-8").splitlines()
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


def config_urdf_path(config_path: str | Path | None) -> Path | None:
    """Return the URDF declared by an arm YAML config, if present."""
    if config_path is None:
        return None

    value = _yaml_top_level_scalar(config_path, "urdf_path")
    if value is None:
        return None
    return resolve_resource_path(value, base=Path(config_path).expanduser().parent)


def config_end_effector_frame(config_path: str | Path | None) -> str | None:
    """Return the default end-effector frame declared by an arm YAML config."""
    if config_path is None:
        return None
    return _yaml_top_level_scalar(config_path, "end_effector_frame")


def resolve_resource_path(path: str | Path, *, base: str | Path | None = None) -> Path:
    """Resolve a repository resource path.

    YAML configs copied from ``reBotArm_control_py`` store URDFs relative to the
    repository root. Keep that convention, while also accepting paths relative
    to the config file for local overrides.
    """
    p = Path(path).expanduser()
    if p.is_absolute():
        return p

    candidates: list[Path] = []
    if base is not None:
        candidates.append(Path(base).expanduser() / p)
    candidates.extend([repo_root() / p, Path.cwd() / p, p])
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def default_calibration_dir() -> Path:
    root = repo_root()
    if (root / "pyproject.toml").exists():
        return root / "calibration"

    cwd = Path.cwd().resolve()
    for parent in [cwd, *cwd.parents]:
        candidates = [
            parent / "rebotarm_control_rt" / "calibration",
            parent / "rebot_lerobot" / "rebotarm_control_rt" / "calibration",
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate

    return Path.cwd() / "calibration"


def resolve_urdf_path(urdf_path: str | Path | None = None) -> Path:
    """Resolve a URDF argument.

    ``None`` returns the original project URDF. Explicit existing paths are
    honored as-is. If the given path does not exist, the SDK-level
    ``calibration/`` directory is searched by filename. For example, both
    ``tool_calibration.urdf`` and ``some/package/tool_calibration.urdf`` resolve
    to ``calibration/tool_calibration.urdf`` when that file exists.
    """
    if urdf_path is None:
        return default_urdf_path()

    path = Path(urdf_path).expanduser()
    if path.is_absolute():
        return path

    calibration_path = default_calibration_dir() / path.name
    if calibration_path.exists():
        return calibration_path

    if path.exists():
        return path

    repo_path = repo_root() / path
    if repo_path.exists():
        return repo_path

    return path


__all__ = [
    "package_root",
    "repo_root",
    "default_urdf_path",
    "default_calibration_dir",
    "config_urdf_path",
    "config_end_effector_frame",
    "resolve_resource_path",
    "resolve_urdf_path",
]
