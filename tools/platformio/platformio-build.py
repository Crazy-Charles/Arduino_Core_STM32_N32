# Copyright 2014-present PlatformIO <contact@platformio.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from os.path import isfile, isdir, join

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()
platform = env.PioPlatform()
board = env.BoardConfig()

FRAMEWORK_DIR = platform.get_package_dir("framework-arduino-n32g45x")
CMSIS_DIR = join(platform.get_package_dir("framework-cmsis"), "CMSIS")
assert isdir(FRAMEWORK_DIR)
assert isdir(CMSIS_DIR)


mcu = env.BoardConfig().get("build.mcu", "")
board_name = env.subst("$BOARD")
variant = board.get("build.variant")
variants_dir = (
    join("$PROJECT_DIR", board.get("build.variants_dir"))
    if board.get("build.variants_dir", "")
    else join(FRAMEWORK_DIR, "variants")
)
variant_dir = join(variants_dir, variant)
upload_protocol = env.subst("$UPLOAD_PROTOCOL")

def process_standard_library_configuration(cpp_defines):
    if "PIO_FRAMEWORK_ARDUINO_STANDARD_LIB" in cpp_defines:
        env["LINKFLAGS"].remove("--specs=nano.specs")
    if "PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_PRINTF" in cpp_defines:
        env.Append(LINKFLAGS=["-u_printf_float"])
    if "PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF" in cpp_defines:
        env.Append(LINKFLAGS=["-u_scanf_float"])


def process_usart_configuration(cpp_defines):
    if "PIO_FRAMEWORK_ARDUINO_SERIAL_DISABLED" in cpp_defines:
        env["CPPDEFINES"].remove("HAL_UART_MODULE_ENABLED")

    elif "PIO_FRAMEWORK_ARDUINO_SERIAL_WITHOUT_GENERIC" in cpp_defines:
        env.Append(CPPDEFINES=["HWSERIAL_NONE"])


def process_usb_speed_configuration(cpp_defines):
    if "PIO_FRAMEWORK_ARDUINO_USB_HIGHSPEED" in cpp_defines:
        env.Append(CPPDEFINES=["USE_USB_HS"])

    elif "PIO_FRAMEWORK_ARDUINO_USB_HIGHSPEED_FULLMODE" in cpp_defines:
        env.Append(CPPDEFINES=["USE_USB_HS", "USE_USB_HS_IN_FS"])


def process_usb_configuration(cpp_defines):
    if "PIO_FRAMEWORK_ARDUINO_ENABLE_CDC" in cpp_defines:
        env.Append(CPPDEFINES=["USBD_USE_CDC"])

    elif "PIO_FRAMEWORK_ARDUINO_ENABLE_CDC_WITHOUT_SERIAL" in cpp_defines:
        env.Append(CPPDEFINES=["USBD_USE_CDC", "DISABLE_GENERIC_SERIALUSB"])

    elif "PIO_FRAMEWORK_ARDUINO_ENABLE_HID" in cpp_defines:
        env.Append(CPPDEFINES=["USBD_USE_HID_COMPOSITE"])

    if any(f in env["CPPDEFINES"] for f in ("USBD_USE_CDC", "USBD_USE_HID_COMPOSITE")):
        env.Append(CPPDEFINES=["HAL_PCD_MODULE_ENABLED"])

env.Append(
    ASFLAGS=["-x", "assembler-with-cpp"],
    CFLAGS=["-std=gnu11"],
    CXXFLAGS=[
        "-std=gnu++14",
        "-fno-threadsafe-statics",
        "-fno-rtti",
        "-fno-exceptions",
        "-fno-use-cxa-atexit",
    ],
    CCFLAGS=[
        "-Os",  # optimize for size
        "-mcpu=%s" % env.BoardConfig().get("build.cpu"),
        "-mthumb",
        "-ffunction-sections",  # place each function in its own section
        "-fdata-sections",
        "-Wall",
        "-nostdlib",
        "--param",
        "max-inline-insns-single=500",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard",
    ],
    CPPDEFINES=[
        "STM32F1xx",
        "STM32F103xE",
        ("ARDUINO", 10808),
        "ARDUINO_%s" % board_name.upper(),
        ("BOARD_NAME", '\\"%s\\"' % board_name.upper()),
        "HAL_UART_MODULE_ENABLED",
    ],
    CPPPATH=[
        join(FRAMEWORK_DIR, "cores", "arduino", "avr"),
        join(FRAMEWORK_DIR, "cores", "arduino", "N32G45x"),
        join(FRAMEWORK_DIR, "cores", "arduino", "N32G45x", "usb"),
        join(FRAMEWORK_DIR, "cores", "arduino", "N32G45x", "usb", "hid"),
        join(FRAMEWORK_DIR, "cores", "arduino", "N32G45x", "usb", "cdc"),
        join(FRAMEWORK_DIR, "system", "Drivers", "N32G45x_HAL_Driver", "Inc"),
        join(FRAMEWORK_DIR, "system", "Drivers", "N32G45x_HAL_Driver", "Src"),
        join(FRAMEWORK_DIR, "system", "N32G45x"),
        join(CMSIS_DIR, "Core", "Include"),
        join(
            FRAMEWORK_DIR,
            "system",
            "Drivers",
            "CMSIS",
            "Include",
        ),
        join(
            FRAMEWORK_DIR,
            "system",
            "Drivers",
            "CMSIS",
            "Source",
            "Templates",
            "gcc",
        ),
        join(
            FRAMEWORK_DIR,
            "system",
            "Middlewares",
            "ST",
            "STM32_USB_Device_Library",
            "Core",
            "Inc",
        ),
        join(
            FRAMEWORK_DIR,
            "system",
            "Middlewares",
            "ST",
            "STM32_USB_Device_Library",
            "Core",
            "Src",
        ),
        join(CMSIS_DIR, "DSP", "Include"),
        join(FRAMEWORK_DIR, "cores", "arduino"),
        variant_dir,
    ],
    LINKFLAGS=[
        "-Os",
        "-mthumb",
        "-mcpu=%s" % env.BoardConfig().get("build.cpu"),
        "--specs=nano.specs",
        "-Wl,--gc-sections,--relax",
        "-Wl,--check-sections",
        "-Wl,--entry=Reset_Handler",
        "-Wl,--unresolved-symbols=report-all",
        "-Wl,--warn-common",
        "-Wl,--defsym=LD_MAX_SIZE=%d" % board.get("upload.maximum_size"),
        "-Wl,--defsym=LD_MAX_DATA_SIZE=%d" % board.get("upload.maximum_ram_size"),
        "-Wl,--defsym=LD_FLASH_OFFSET=0x0",
        "-mfpu=fpv4-sp-d16", 
        "-mfloat-abi=hard",
    ],
    LIBS=[
        "arm_cortexM4lf_math",
        "c",
        "m",
        "gcc",
        "stdc++",
    ],
    LIBPATH=[variant_dir, join(CMSIS_DIR, "DSP", "Lib", "GCC")],
)

#
# Linker requires preprocessing with correct RAM|ROM sizes
#

if not board.get("build.ldscript", ""):
    env.Replace(LDSCRIPT_PATH=join(FRAMEWORK_DIR, "system", "ldscript.ld"))
    if not isfile(join(env.subst(variant_dir), "ldscript.ld")):
        print("Warning! Cannot find linker script for the current target!\n")
    env.Append(LINKFLAGS=[("-Wl,--default-script", join(variant_dir, "ldscript.ld"))])

#
# Process configuration flags
#

cpp_defines = env.Flatten(env.get("CPPDEFINES", []))

process_standard_library_configuration(cpp_defines)
process_usb_configuration(cpp_defines)
process_usb_speed_configuration(cpp_defines)
process_usart_configuration(cpp_defines)

# copy CCFLAGS to ASFLAGS (-x assembler-with-cpp mode)
env.Append(ASFLAGS=env.get("CCFLAGS", [])[:])

env.Append(
    LIBSOURCE_DIRS=[
        join(FRAMEWORK_DIR, "libraries"),
    ]
)

#
# Target: Build Core Library
#

libs = []

if "build.variant" in env.BoardConfig():
    env.Append(CPPPATH=[variant_dir])
    env.BuildSources(join("$BUILD_DIR", "Variant"), variant_dir)

env.BuildSources(
    join("$BUILD_DIR", "FrameworkArduino"), join(FRAMEWORK_DIR, "cores", "arduino")
)

env.BuildSources(
    join("$BUILD_DIR", "CoreDrivers"), join(FRAMEWORK_DIR, "libraries", "CoreDrivers")
)

env.Prepend(LIBS=libs)
