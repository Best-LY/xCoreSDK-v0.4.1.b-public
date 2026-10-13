cmake_minimum_required(VERSION 3.12)
project(rokae_examples)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Threads REQUIRED)  ##  新增

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib/Linux/x86_64)

# --- 3. 自动化构建与目标属性设置 (现代化和优化) ---
file(GLOB EXAMPLE_SOURCES "*.cpp")
foreach(source_file ${EXAMPLE_SOURCES})
    get_filename_component(executable_name ${source_file} NAME_WE)
    add_executable(${executable_name} ${source_file})

    # 替换 include_directories：使用目标特定的包含路径
    # PRIVATE 表示只有这个可执行文件需要这些头文件路径
    target_include_directories(${executable_name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${OpenCV_INCLUDE_DIRS}
        ${realsense2_INCLUDE_DIRS}
    )

    # 替换 CXX_FLAGS 中的 -pthread：使用 Threads 库链接
    target_link_libraries(${executable_name}
        ${OpenCV_LIBS}
        eigen
        realsense2::realsense2
        xCoreSDK
        xMateModel
        Threads::Threads # 使用现代 CMake 链接方式添加多线程支持
    )
endforeach()



# include_directories(
#     ${CMAKE_CURRENT_SOURCE_DIR}
#     ${CMAKE_CURRENT_SOURCE_DIR}/../include
#     ${OpenCV_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIRS}
#     ${realsense2_INCLUDE_DIRS}
# )

# file(GLOB EXAMPLE_SOURCES "*.cpp")
# foreach(source_file ${EXAMPLE_SOURCES})
#     get_filename_component(executable_name ${source_file} NAME_WE)
#     add_executable(${executable_name} ${source_file})
#     target_link_libraries(${executable_name}
#         ${OpenCV_LIBS}
#         ${EIGEN3_LIBRARIES}
#         realsense2::realsense2
#         xCoreSDK
#         xMateModel
#         Threads::Threads # 使用现代 CMake 链接方式添加多线程支持
#     )
# endforeach()

# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rt/CMakeLists.txt")
#     add_subdirectory(rt)
# endif()
