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
        shutil.copy(hex_file, os.path.join(dest_dir, "firmware.hex"))
    
    print(f"Firmware copied to {dest_dir}")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "ihex", "-R", ".eeprom",
        "$BUILD_DIR/${PROGNAME}.elf", "$BUILD_DIR/${PROGNAME}.hex"
    ]), "Building $BUILD_DIR/${PROGNAME}.hex")
)

# Run after the .elf (and thus after .hex and .bin are generated)
env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", after_build)
