Import("env")
import os
import shutil

def after_build(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    progname = env.subst("$PROGNAME")
    project_dir = env.subst("$PROJECT_DIR")
    dest_dir = os.path.join(project_dir, "compiled_firmware")
    os.makedirs(dest_dir, exist_ok=True)

    bin_file = os.path.join(build_dir, f"{progname}.bin")
    hex_file = os.path.join(build_dir, f"{progname}.hex")

    if os.path.exists(bin_file):
        shutil.copy(bin_file, os.path.join(dest_dir, "firmware.bin"))
    if os.path.exists(hex_file):
        # Keep the tracked Intel HEX portable and free of CRLF whitespace.
        with open(hex_file, "r", newline="") as source_file:
            hex_data = source_file.read().replace("\r\n", "\n").replace("\r", "\n")
        with open(os.path.join(dest_dir, "firmware.hex"), "w", newline="\n") as dest_file:
            dest_file.write(hex_data)

    print(f"Firmware copied to {dest_dir}")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "ihex", "-R", ".eeprom",
        "$BUILD_DIR/${PROGNAME}.elf", "$BUILD_DIR/${PROGNAME}.hex"
    ]), "Building $BUILD_DIR/${PROGNAME}.hex")
)

# BIN is generated after the custom HEX action, so both artifacts are ready here.
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", after_build)
