cmake_minimum_required(VERSION 3.14)
project(CBS_Localized_Conflict_Resolution)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# 编译选项
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Google Test 配置
include(FetchContent)
# FetchContent_Declare(
#     googletest
#     GIT_REPOSITORY https://github.com/google/googletest.git
#     GIT_TAG release-1.12.1
# )
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://gitee.com/mirrors/googletest.git
    GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)
enable_testing()

# 收集源文件
file(GLOB_RECURSE PROJECT_SOURCES 
    "src/*.cpp"
    "src/*.h"
    "data_loader/*.cpp"
    "data_loader/*.h"
    "benchmark/*.cpp"
    "benchmark/*.h"
)

# 创建库
add_library(project_lib ${PROJECT_SOURCES})
target_include_directories(project_lib PUBLIC ${CMAKE_SOURCE_DIR})


# 创建test目录
add_subdirectory(test)

# 添加Google Test
add_executable(preprocess_test test/PreprocessTest.cpp)
target_link_libraries(preprocess_test PRIVATE project_lib gtest gtest_main)

add_executable(preprocess_detailed_test test/PreprocessDetailedTest.cpp)
target_link_libraries(preprocess_detailed_test PRIVATE project_lib gtest gtest_main)

add_executable(key_interval_astar_test test/KeyIntervalAStarTest.cpp)
target_link_libraries(key_interval_astar_test PRIVATE project_lib gtest gtest_main)

# 启用测试
include(GoogleTest)
gtest_discover_tests(preprocess_test)
gtest_discover_tests(preprocess_detailed_test)
gtest_discover_tests(key_interval_astar_test)

# 添加Key Interval A* benchmark可执行文件
add_executable(single_test benchmark/SingleTest.cpp)
target_link_libraries(single_test PRIVATE project_lib)

add_executable(benchmark benchmark/Benchmark.cpp)
target_link_libraries(benchmark PRIVATE project_lib)

