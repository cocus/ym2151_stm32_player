import os
import logging
from xml.etree.ElementInclude import include

Import("env")

logging.basicConfig(level=logging.DEBUG)

# global build environment is for all libraries and code
global_env = DefaultEnvironment()

ProjectDir = env.subst("$PROJECT_DIR")
BuildDir = env.subst("$BUILD_DIR")
MiddlewaresDirectory = "Middlewares"
DriversDirectory = "Drivers"
UsbDeviceDirectory = "USB_DEVICE"

def process_cubemxstuff():
    logging.debug("process cubemx stuff!")
    add_core_stuff()
    process_hal()
    process_middlewares()
    process_usbdevice()

def add_core_stuff():
    include_parts = []

    include_parts.append(os.path.join(ProjectDir, "Core", "Inc"))

    for inc in include_parts:
        logging.debug("CubeMX[Core]: adding new include {0}".format(inc))
        env.Append(CPPPATH=[os.path.realpath(inc)])
        global_env.Append(CPPPATH=[os.path.realpath(inc)])

    for (dirpath, dirnames, filenames) in os.walk(os.path.join(ProjectDir, "Core", "Startup")):
        for file in filenames:
            if "startup_" in file:
                logging.debug("CubeMX[Core]: adding startup {0}".format(file))
                env.BuildSources(os.path.join(BuildDir, "Startup"), os.path.realpath(dirpath))

    for (dirpath, dirnames, filenames) in os.walk(os.path.join(ProjectDir, "STM32CubeIDE", "Application", "User", "Startup")):
        for file in filenames:
            if "startup_" in file:
                logging.debug("CubeMX[Core]: adding startup {0}".format(file))
                env.BuildSources(os.path.join(BuildDir, "Startup"), os.path.realpath(dirpath))

    for (dirpath, dirnames, filenames) in os.walk(os.path.join(ProjectDir, "STM32CubeIDE", "Application", "User", "Core")):
        for file in filenames:
            if "syscalls" in file:
                logging.debug("CubeMX[Core]: adding syscalls!!!!!! {0}".format(file))
                env.BuildSources(os.path.join(BuildDir, "Core"), os.path.realpath(dirpath))

def process_hal():
    DriversPath = os.path.join(ProjectDir, DriversDirectory)

    logging.debug("CubeMX: Drivers path: {0}".format(DriversPath))

    for (dirpath, dirnames, filenames) in os.walk(DriversPath):
        for dir in dirnames:
            if "CMSIS" in dir:
                process_driver_cmsis(os.path.join(dirpath, dir).replace(ProjectDir + os.path.sep, ""))
            if "HAL_Driver" in dir:
                process_driver_hal(os.path.join(dirpath, dir).replace(ProjectDir + os.path.sep, ""))

def process_middlewares():
    MiddlewaresPath = os.path.join(ProjectDir, MiddlewaresDirectory)

    logging.debug("CubeMX: Middlewares path: {0}".format(MiddlewaresPath))

    for (dirpath, dirnames, filenames) in os.walk(MiddlewaresPath):
        if "FreeRTOS" in dirnames:
            process_middleware_freertos(os.path.join(dirpath, "FreeRTOS").replace(ProjectDir + os.path.sep, ""))
        if "STM32_USB_Device_Library" in dirnames:
            process_middleware_st_usb_device(os.path.join(dirpath, "STM32_USB_Device_Library").replace(ProjectDir + os.path.sep, ""))
        if "FatFs" in dirnames:
            process_middleware_st_fatfs(os.path.join(dirpath, "FatFs").replace(ProjectDir + os.path.sep, ""))

def process_driver_cmsis(source_path):
    logging.debug("CubeMX[CMSIS]: Source path for CMSIS: {0}".format(source_path))

    device_st_dir = os.path.join(source_path, "Device", "ST")
    device_st_include_dir = None
    for (dirpath, dirnames, filenames) in os.walk(device_st_dir):
        if "Include" in dirnames:
            device_st_include_dir = os.path.join(dirpath, "Include")
            break

    if device_st_include_dir is None:
        logging.debug("CubeMX[CMSIS]: can't find device include dir!")
        exit(1)

    include_parts = []

    include_parts.append(device_st_include_dir)
    logging.debug("CubeMX[CMSIS]: device include dir: {0}".format(device_st_include_dir))
    include_parts.append(os.path.join(source_path, "Include"))

    for inc in include_parts:
        logging.debug("CubeMX[CMSIS]: adding new include {0}".format(inc))
        global_env.Append(CPPPATH=[os.path.realpath(inc)])

def process_driver_hal(source_path):
    logging.debug("CubeMX[HAL]: Source path for HAL: {0}".format(source_path))

    source_dir = os.path.join(source_path, "Src")
    absolute_build_dir = os.path.join(BuildDir, source_dir)
    absolute_source_dir = os.path.join(ProjectDir, source_dir)

    logging.debug("CubeMX[HAL]: processing: {0}, {1}".format(absolute_build_dir, absolute_source_dir))

    include_parts = []

    include_parts.append(os.path.join(source_path, "Inc"))
    include_parts.append(os.path.join(source_path, "Inc", "Legacy"))

    for inc in include_parts:
        logging.debug("CubeMX[HAL]: adding new include {0}".format(inc))
        env.Append(CCFLAGS=["-DOS_USE_SEMIHOSTING"])
        env.Append(CPPPATH=[os.path.realpath(inc)])
        global_env.Append(CCFLAGS=["-DOS_USE_SEMIHOSTING"])
        global_env.Append(CPPPATH=[os.path.realpath(inc)])

    env.BuildSources(absolute_build_dir, source_dir)

def process_middleware_freertos(source_path):
    logging.debug("CubeMX[FreeRTOS]: Source path for freertos: {0}".format(source_path))

    source_dir = os.path.join(source_path, "Source")
    absolute_build_dir = os.path.join(BuildDir, source_dir)
    absolute_source_dir = os.path.join(ProjectDir, source_dir)

    logging.debug("CubeMX[FreeRTOS]: processing: {0}, {1}".format(absolute_build_dir, absolute_source_dir))

    include_parts = []

    include_parts.append(os.path.join(source_dir, "include"))
    include_parts.append(os.path.join(source_dir, "CMSIS_RTOS"))
    include_parts.append(os.path.join(source_dir, "portable", "GCC", "ARM_CM3"))
    include_parts.append(os.path.join(source_dir, "portable", "MemMang"))

    for inc in include_parts:
        logging.debug("CubeMX[FreeRTOS]: adding new include {0}".format(inc))
        env.Append(CPPPATH=[os.path.realpath(inc)])
        global_env.Append(CPPPATH=[os.path.realpath(inc)])

    env.BuildSources(absolute_build_dir, source_dir)

def process_middleware_st_usb_device(source_path):
    logging.debug("CubeMX[ST_USB_DEV]: Source path for usb device: {0}".format(source_path))

    source_dir = os.path.join(source_path, "Core", "Src")
    absolute_build_dir = os.path.join(BuildDir, "external", "build", "stusbdev")
    env.BuildSources(absolute_build_dir, source_dir)

    source_dir = os.path.join(source_path, "Class", "CDC", "Src")
    absolute_build_dir = os.path.join(BuildDir, "external", "build", "stusbdev", "cdc")
    env.BuildSources(absolute_build_dir, source_dir)

    include_parts = []

    include_parts.append(os.path.join(source_path, "Core", "Inc"))
    include_parts.append(os.path.join(source_path, "Class", "CDC", "Inc"))

    for inc in include_parts:
        logging.debug("CubeMX[FreeRTOS]: adding new include {0}".format(inc))
        env.Append(CPPPATH=[os.path.realpath(inc)])
        global_env.Append(CPPPATH=[os.path.realpath(inc)])

def process_middleware_st_fatfs(source_path):
    logging.debug("CubeMX[ST_FATFS]: Source path for fatfs: {0}".format(source_path))

    source_dir = os.path.join(source_path, "src")
    absolute_build_dir = os.path.join(BuildDir, source_dir)
    absolute_source_dir = os.path.join(ProjectDir, source_dir)

    include_parts = []

    include_parts.append(os.path.join(source_dir))
    include_parts.append(os.path.join("FATFS", "Target"))
    include_parts.append(os.path.join("FATFS", "App"))

    for inc in include_parts:
        logging.debug("CubeMX[FreeRTOS]: adding new include {0}".format(inc))
        env.Append(CPPPATH=[os.path.realpath(inc)])
        global_env.Append(CPPPATH=[os.path.realpath(inc)])

    env.BuildSources(absolute_build_dir, source_dir)
    #env.BuildSources(os.path.join(absolute_build_dir, "option"), os.path.join(source_dir, "option"))

    env.BuildSources(os.path.join(absolute_build_dir, "FATFS", "Target"), os.path.join("FATFS", "Target"))
    env.BuildSources(os.path.join(absolute_build_dir, "FATFS", "App"), os.path.join("FATFS", "App"))

def process_usbdevice():
    UsbDevicePath = os.path.join(ProjectDir, UsbDeviceDirectory)

    if (not os.path.exists(UsbDevicePath)):
        return

    logging.debug("CubeMX: USB Device path: {0}".format(UsbDevicePath))

    absolute_build_dir = os.path.join(BuildDir, UsbDeviceDirectory)

    include_parts = []

    include_parts.append(os.path.join(UsbDeviceDirectory, "App"))
    include_parts.append(os.path.join(UsbDeviceDirectory, "Target"))

    for inc in include_parts:
        logging.debug("CubeMX[FreeRTOS]: adding new include {0}".format(inc))
        env.Append(CPPPATH=[os.path.realpath(inc)])
        global_env.Append(CPPPATH=[os.path.realpath(inc)])

    env.BuildSources(os.path.join(absolute_build_dir, "App"), os.path.join(UsbDeviceDirectory, "App"))
    env.BuildSources(os.path.join(absolute_build_dir, "Target"), os.path.join(UsbDeviceDirectory, "Target"))

process_cubemxstuff()
