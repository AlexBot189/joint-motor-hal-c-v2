cmake_minimum_required(VERSION 3.10)
project(stark_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# algo_sim — 模拟算法进程
add_executable(algo_sim algo_sim.cpp)
target_include_directories(algo_sim PRIVATE ..)
target_link_libraries(algo_sim rt pthread)

# perf_test — SHM 性能测试
add_executable(perf_test perf_test.cpp)
target_include_directories(perf_test PRIVATE ..)
target_link_libraries(perf_test rt pthread)

# stark_tool — CLI 调试工具 (纯C, 直连SHM)
set_source_files_properties(stark_tool.c PROPERTIES LANGUAGE C)
add_executable(stark_tool stark_tool.c)
target_include_directories(stark_tool PRIVATE ..)
target_link_libraries(stark_tool rt pthread)

# demo_algo — 算法控制示例 (纯C)
set_source_files_properties(demo_algo.c PROPERTIES LANGUAGE C)
add_executable(demo_algo demo_algo.c)
target_include_directories(demo_algo PRIVATE ..)
target_link_libraries(demo_algo rt pthread m)
