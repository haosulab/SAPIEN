import pkg_resources
from warnings import warn
import platform
import os


def _ensure_libvulkan():
    # find and use system vulkan
    LD_LIBRARY_PATH = os.environ.get("LD_LIBRARY_PATH", "")
    link_paths = [x.strip() for x in LD_LIBRARY_PATH.split(":") if x.strip()]
    extra_paths = ["/usr/lib", "/usr/lib64", "/usr/lib/x86_64-linux-gnu"]
    for path in link_paths + extra_paths:
        if os.path.isfile(os.path.join(path, "libvulkan.so.1")):
            return

    # add our vulkan to LD_LIBRARY_PATH
    vulkan_library_path = pkg_resources.resource_filename(
        "sapien", "vulkan_library/libvulkan.so.1.3.224"
    )

    warn("Failed to find system libvulkan. Fallback to SAPIEN builtin libvulkan.")
    os.environ["SAPIEN_VULKAN_LIBRARY_PATH"] = vulkan_library_path


def _ensure_vulkan_icd():
    if os.system("nvidia-smi > /dev/null 2>&1") != 0:
        return

    if os.environ.get("VK_ICD_FILENAMES"):
        return

    if os.path.exists("/usr/share/vulkan/icd.d") and os.path.isfile(
        "/usr/share/vulkan/icd.d/nvidia_icd.json"
    ):
        return

    warn(
        "Failed to find Vulkan ICD file. This is probably due to an incorrect or partial installation of the NVIDIA driver. SAPIEN will attempt to provide an ICD file anyway but it may not work."
    )
    os.environ["VK_ICD_FILENAMES"] = pkg_resources.resource_filename(
        "sapien", "vulkan_library/nvidia_icd.json"
    )


def _ensure_egl_icd():
    if os.system("nvidia-smi > /dev/null 2>&1") != 0:
        return

    if os.environ.get("__EGL_VENDOR_LIBRARY_FILENAMES") or os.environ.get(
        "__EGL_VENDOR_LIBRARY_DIRS"
    ):
        return

    # 10_nvidia.json is installed
    for d in ["/usr/share/glvnd/egl_vendor.d", "/etc/glvnd/egl_vendor.d"]:
        if any(("nvidia" in f) for f in os.listdir(d)):
            return

    warn(
        "Failed to find glvnd ICD file. This is probably due to an incorrect or partial installation of the NVIDIA driver. SAPIEN will attempt to provide an ICD file anyway but it may not work."
    )

    os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = pkg_resources.resource_filename(
        "sapien", "vulkan_library/10_nvidia.json"
    )


if platform.system() == "Linux":
    _ensure_libvulkan()
    _ensure_vulkan_icd()
    _ensure_egl_icd()
