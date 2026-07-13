cmake_minimum_required(VERSION 3.10)
project(Umbra)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Search paths for FRC libs manually installed in /usr/local
include_directories(/usr/local/include)
include_directories(/usr/include/apriltag)
link_directories(/usr/local/lib)

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(Umbra main.cpp)

target_link_libraries(Umbra 
    ${OpenCV_LIBS} 
    nlohmann_json::nlohmann_json
    ntcore wpiutil wpinet apriltag
)
