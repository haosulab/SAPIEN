import ctypes
import platform
import os

if platform.system() == "Linux":
    try:
        oidn_dll = ctypes.CDLL(
            os.path.join(
                os.path.dirname(__file__),
                "oidn_library/libOpenImageDenoise_core.so.2.0.1",
            ),
            ctypes.RTLD_LOCAL,
        )

        oidn_dll = ctypes.CDLL(
            os.path.join(
                os.path.dirname(__file__), "oidn_library/libOpenImageDenoise.so.2.0.1"
            ),
            ctypes.RTLD_LOCAL,
        )
    except Exception:
        pass
