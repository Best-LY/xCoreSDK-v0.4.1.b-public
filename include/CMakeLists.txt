# cmake_minimum_required(VERSION 3.0.2)
# project(rokae_include)

# # Set CMake policies
# cmake_policy(SET CMP0079 NEW)

# # Find required packages
# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(realsense2 REQUIRED)

# # Create interface library with a unique name
# add_library(${PROJECT_NAME} INTERFACE)

# # Specify include directories for interface library
# target_include_directories(${PROJECT_NAME} 
#     INTERFACE
#         ${CMAKE_CURRENT_SOURCE_DIR}
#         ${OpenCV_INCLUDE_DIRS}
#         ${EIGEN3_INCLUDE_DIRS}
#         ${realsense2_INCLUDE_DIRS}
# )

# # Link libraries
# target_link_libraries(${PROJECT_NAME}
#     INTERFACE
#         ${OpenCV_LIBS}
#         ${EIGEN3_LIBRARIES}
#         realsense2::realsense2
# )