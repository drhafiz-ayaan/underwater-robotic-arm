#!/usr/bin/env python3
"""
Offscreen GPU rendering setup, shared by the launch files.

WHY THIS IS NEEDED
On a laptop with both an Intel iGPU and an NVIDIA dGPU, libglvnd ranks EGL
vendors by filename: 10_nvidia.json wins over 50_mesa.json. Gazebo's headless
sensor path then enumerates the NVIDIA DRM render node but the loader resolves
Mesa for it, and Mesa has no driver for an Ampere PCI ID. The result is:

    libEGL warning: pci id for fd N: 10de:249c, driver (null)
    libEGL warning: egl: failed to create dri2 screen
    MESA: warning: Driver does not support the 0x249c PCI ID

Rendering then falls back to a path that clears to the background colour but
never rasterises geometry - camera topics publish, every frame is flat, and
nothing announces that it went wrong. Pinning the NVIDIA vendor explicitly fixes
it and puts sensor rendering on the dGPU.

Detection is by file presence, so this is a no-op on machines without the
NVIDIA EGL vendor installed.
"""

import os

from launch.actions import SetEnvironmentVariable

NVIDIA_EGL_VENDOR = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"


def gpu_env():
    """Return SetEnvironmentVariable actions pinning EGL to the NVIDIA dGPU."""
    if not os.path.exists(NVIDIA_EGL_VENDOR):
        return []
    return [
        SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES", NVIDIA_EGL_VENDOR),
        SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
        SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
    ]
