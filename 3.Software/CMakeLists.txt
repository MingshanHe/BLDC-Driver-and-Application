# # License: Apache 2.0. See LICENSE file in root directory.
# # Copyright(c) 2019 Intel Corporation. All Rights Reserved.
# cmake_minimum_required(VERSION 3.1.0)

# project(realsense2)

# # Find librealsense2 installed package
# find_package(realsense2 REQUIRED)
# find_package(OpenGL REQUIRED)
# find_package(glfw3)
# find_package (OpenCV REQUIRED)

# message(STATUS "OpenCV library status:")
# message(STATUS "    config: ${OpenCV_DIR}")
# message(STATUS "    version: ${OpenCV_VERSION}")
# message(STATUS "    libraries: ${OpenCV_LIBS}")
# message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


# set(DEPENDENCIES    realsense2 
#                     glfw
#                     ${OpenCV_LIBRARY_DIRS}
#                     ${OPENGL_LIBRARIES}
#                     ${OpenCV_LIBS})

# include_directories(include 
#                     third_party/tclap/include
#                     third_party/imgui
#                     third_party
                    
#                     "/usr/include/opencv4/")
# include_directories(
#     ${OpenCV_INCLUDE_DIRS}
#     )
# # Enable C++11
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# # Add the application sources to the target
# add_executable(hello_librealsense2 src/hello_librealsense2.cpp)
# add_executable(rs_capture src/rs_capture.cpp)
# add_executable(rs_color src/rs_color.cpp)
# add_executable(rs_visualize src/rs_visualize.cpp)

# set_property(TARGET rs_capture PROPERTY CXX_STANDARD 11)

# # Link librealsense2 to the target
# link_directories(${OpenCV_LIBRARY_DIRS})

# target_link_libraries(hello_librealsense2 ${realsense2_LIBRARY})
# target_link_libraries(rs_capture ${DEPENDENCIES} ${realsense2_LIBRARY} OpenGL::OpenGL glfw)
# target_link_libraries(rs_color ${DEPENDENCIES} ${realsense2_LIBRARY})
# # target_link_libraries(rs_visualize ${DEPENDENCIES} ${realsense2_LIBRARY} ${OpenCV_LIBRARY_DIRS} ${OpenCV_LIBS})
# # 库文件
# link_directories(${OpenCV_LIBRARY_DIRS})
# target_link_libraries(rs_visualize ${OpenCV_LIBS})

# install(TARGETS rs_capture RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
cmake_minimum_required(VERSION 3.1.0)

PROJECT(opencvrealsense)

find_package(OpenCV REQUIRED) 


SET(PROJECT_INCLUDE ${PROJECT_SOURCE_DIR}/source)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} ${PROJECT_INCLUDE})



add_executable(rs_visualize src/rs_visualize.cpp)
target_link_libraries(rs_visualize ${OpenCV_LIBS} realsense2)