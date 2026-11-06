# host 对拍测试：把 control_framework_v2 的 C 实现和 three_friction 的 C++ 实现
# 用同一组输入跑一遍，比较输出。不参与固件构建。
cmake_minimum_required(VERSION 3.16)
project(three_friction_host_tests C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 导出编译数据库，供 clangd/编辑器解析 host 测试的 include 路径
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED V2_ROOT)
    set(V2_ROOT "/home/ye/Projects/embedded/control_framework_v2/UserCode" CACHE PATH "control_framework_v2/UserCode")
endif()

set(FW_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(STUBS   ${CMAKE_CURRENT_SOURCE_DIR}/stubs)
set(CASES   ${CMAKE_CURRENT_SOURCE_DIR}/cases)

# ── host 桩（ref 与 new 共用同一套矩阵/DWT 替身）──
add_library(host_stubs STATIC
    ${STUBS}/arm_math_stub.cpp
    ${STUBS}/hal_dwt_host.cpp
)
target_include_directories(host_stubs PUBLIC ${STUBS})

# ── 参考实现（v2 的 C 代码，原样编译）──
add_library(ref_algo STATIC
    ${V2_ROOT}/Common/utils.c
    ${V2_ROOT}/Common/controller.c
    ${V2_ROOT}/Common/kalman_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ref/QuaternionEKF_host.c
    ${V2_ROOT}/Common/crc8_crc16.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cases/ref_wrap.c
)
target_include_directories(ref_algo PUBLIC ${STUBS} ${V2_ROOT}/Common)
target_link_libraries(ref_algo PUBLIC host_stubs)
target_compile_options(ref_algo PRIVATE -w -include string.h)   # 老 C 代码隐式用到 memcpy/memset，切告警不参与判定

# ── 新的 C++ 实现（按已存在的文件自动收集）──
set(NEW_SRCS "")
# dwt.cpp 是纯固件实现（直接读 DWT 寄存器），host 用 stubs/dwt_host.cpp 代替
foreach(f ols cotr crc kalman_filter quaternion_ekf)
    if(EXISTS ${FW_ROOT}/UserCode/Common/${f}.cpp)
        list(APPEND NEW_SRCS ${FW_ROOT}/UserCode/Common/${f}.cpp)
    endif()
endforeach()
list(APPEND NEW_SRCS ${STUBS}/dwt_host.cpp)

if(NEW_SRCS)
    add_library(new_algo STATIC ${NEW_SRCS})
    target_include_directories(new_algo PUBLIC ${STUBS} ${FW_ROOT}/UserCode/Common)
    target_include_directories(new_algo PRIVATE ${FW_ROOT}/UserCode)
    target_compile_definitions(new_algo PUBLIC HOST_TEST=1)
    target_compile_options(new_algo PRIVATE -Wall -Wextra -Wshadow)
    target_link_libraries(new_algo PUBLIC host_stubs)
endif()

enable_testing()

function(add_host_test name)
    add_executable(${name} ${CASES}/${name}.cpp)
    target_link_libraries(${name} PRIVATE ref_algo host_stubs)
    if(TARGET new_algo)
        target_link_libraries(${name} PRIVATE new_algo)
    endif()
    target_include_directories(${name} PRIVATE ${STUBS} ${V2_ROOT}/Common ${FW_ROOT}/UserCode/Common)
    target_include_directories(${name} PRIVATE ${FW_ROOT}/UserCode/DRV/motor ${FW_ROOT}/UserCode/HAL)
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_host_test(test_smoke)
foreach(t test_dwt test_ols test_crc test_pid test_kf test_ekf)
    if(EXISTS ${CASES}/${t}.cpp)
        add_host_test(${t})
    endif()
endforeach()
