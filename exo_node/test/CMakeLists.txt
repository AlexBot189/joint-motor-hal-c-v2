cmake_minimum_required(VERSION 3.10)
project(exo_test)

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
