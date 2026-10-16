cmake_minimum_required(VERSION 3.20)

project(Umbra LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(UMBRA_BUILD_APP "Build the camera and NetworkTables application" ON)
option(UMBRA_BUILD_TESTS "Build native mathematical regression tests" OFF)

# The local test build deliberately does not require ARM-only dependencies.
find_package(OpenCV REQUIRED)

add_library(umbra_pose STATIC pose_math.cpp)
target_include_directories(umbra_pose PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(umbra_pose PUBLIC ${OpenCV_LIBS})

if(UMBRA_BUILD_APP)
find_package(nlohmann_json CONFIG REQUIRED)

# Find ARM64 libraries when cross-compiling,
# or local system libraries when building on the Pi
find_library(NTCORE_LIBRARY
    NAMES ntcore
    PATHS /usr/local/lib
    REQUIRED
)

find_library(WPIUTIL_LIBRARY
    NAMES wpiutil
    PATHS /usr/local/lib
    REQUIRED
)

find_library(WPINET_LIBRARY
    NAMES wpinet
    PATHS /usr/local/lib
    REQUIRED
)

find_library(APRILTAG_LIBRARY
    NAMES apriltag
    PATHS /usr/lib/aarch64-linux-gnu /usr/lib
    REQUIRED
)

add_executable(Umbra main.cpp)

# On the Pi CMAKE_SYSROOT is empty.
# In the container it points to our copied DietPi filesystem.
target_include_directories(Umbra PRIVATE
    "${CMAKE_SYSROOT}/usr/local/include"
    "${CMAKE_SYSROOT}/usr/include/apriltag"
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(Umbra PRIVATE
    umbra_pose
    nlohmann_json::nlohmann_json
    ${NTCORE_LIBRARY}
    ${WPIUTIL_LIBRARY}
    ${WPINET_LIBRARY}
    ${APRILTAG_LIBRARY}
)
endif()

if(UMBRA_BUILD_TESTS)
    find_package(nlohmann_json CONFIG REQUIRED)
    enable_testing()
    add_executable(umbra_pose_tests tests/pose_tests.cpp)
    target_link_libraries(umbra_pose_tests PRIVATE umbra_pose nlohmann_json::nlohmann_json)
    target_compile_definitions(umbra_pose_tests PRIVATE
        UMBRA_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    include(tests/CTestCases.cmake)
endif()
