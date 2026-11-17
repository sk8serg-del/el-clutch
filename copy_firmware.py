# После сборки кладёт прошивку в папку своего железа с версией в имени:
# firmware/GEN1_C3/GEN1_v2.16.13_C3.bin, firmware/GEN2_S3/GEN2_v2.16.13_S3.bin,
# firmware/GEN2_C3/GEN2_v2.16.13_C3.bin (до 2.16.13 — CLUTCH_C3, CLUTCH_CAN_S3).
# Версия берётся из FW_VERSION_BASE в прошивке.
import os
import re
import shutil

Import("env")


def copy_firmware(source, target, env):
    project_dir = env.subst("$PROJECT_DIR")
    with open(os.path.join(project_dir, "src", "StarkClutch.ino"), encoding="utf-8") as f:
        version = re.search(r'#define FW_VERSION_BASE "([^"]+)"', f.read()).group(1)
    pioenv = env.subst("$PIOENV").lower()
    board = "S3" if "s3" in pioenv else "C3"
    hw = "GEN2" if (board == "S3" or pioenv.endswith("_can")) else "GEN1"
    out_dir = os.path.join(project_dir, "firmware", "%s_%s" % (hw, board))
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "%s_v%s_%s.bin" % (hw, version, board))
    shutil.copyfile(str(target[0]), out_path)
    print("Firmware saved: " + out_path)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_firmware)
