set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Matches the sysroot mount inside the Podman container
set(CMAKE_SYSROOT "/work/umbra-sysroot")

set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")

# Build tools come from the container
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Headers, libraries and packages come from DietPi
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
