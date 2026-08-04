import ctypes
import sys
from pathlib import Path


_LIBSTDCPP_HANDLE = None


def ensure_compatible_libstdcpp() -> None:
    """Promote libstdc++ into the GLOBAL symbol scope before loading native modules.

    Pinocchio's URDF parser (reached through ``_math``) relies on C++ RTTI/exception
    symbols resolving to a single, process-global libstdc++. CPython dlopen's extension
    modules with ``RTLD_LOCAL``, so if another extension (e.g. torch) already mapped
    libstdc++ into a local scope first, pinocchio's symbol resolution becomes inconsistent
    across the DSO boundary and ``load_robot_model`` segfaults. Re-opening libstdc++ with
    ``RTLD_GLOBAL`` promotes it into the global scope and makes resolution consistent.

    This must use the SONAME, not an absolute path: when libstdc++ is already loaded,
    dlopen'ing a *different* file (e.g. ``$PREFIX/lib/libstdc++.so.6``) by absolute path
    fails to promote the already-active copy in place and can crash. The soname promotes
    whatever is already loaded; the absolute path is only a fallback for when the soname is
    not on the loader search path (nothing has pulled libstdc++ in yet).
    """
    global _LIBSTDCPP_HANDLE
    if _LIBSTDCPP_HANDLE is not None or not sys.platform.startswith("linux"):
        return

    try:
        _LIBSTDCPP_HANDLE = ctypes.CDLL("libstdc++.so.6", mode=ctypes.RTLD_GLOBAL)
        return
    except OSError:
        pass

    libstdcpp = Path(sys.prefix) / "lib" / "libstdc++.so.6"
    if libstdcpp.exists():
        _LIBSTDCPP_HANDLE = ctypes.CDLL(str(libstdcpp), mode=ctypes.RTLD_GLOBAL)
