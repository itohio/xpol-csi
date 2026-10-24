"""PlatformIO pre-script: rebuild web/dist before the SPIFFS image is made,
so `pio run -t uploadfs` always ships the current dashboard."""
Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

import subprocess
from pathlib import Path


def build_web(*_args, **_kwargs):
    web = Path(env.subst("$PROJECT_DIR")) / "web"  # noqa: F821
    subprocess.run(["npm", "--prefix", str(web), "run", "build"], check=True)


env.AddPreAction("$BUILD_DIR/${ESP32_FS_IMAGE_NAME}.bin", build_web)  # noqa: F821
