cmake_minimum_required(VERSION 3.16)
project(AzureKinectDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------ OpenCV ------------------
find_package(OpenCV 4.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# ------------------ PCL ------------------
find_package(PCL 1.11 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ------------------ Azure Kinect SDK ------------------
find_package(k4a REQUIRED)

# ------------------ RealSense ------------------
find_package(realsense2 REQUIRED)

# ------------------ Include Directories ------------------
include_directories(${PROJECT_SOURCE_DIR}/inc)

# ------------------ Source Files ------------------
set(SOURCES
    src/main_rs.cpp
    src/realsense.cpp
    src/yolo.cpp        # 👈 别忘了把 yolo.cpp 加进来
)

# ------------------ Executable ------------------
add_executable(AzureKinectDemo ${SOURCES})

# ------------------ Link Libraries ------------------
target_link_libraries(AzureKinectDemo
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    k4a
    realsense2
)