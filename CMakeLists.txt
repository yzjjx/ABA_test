cmake_minimum_required(VERSION 3.10)
project(ABA_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ABA_test 
    test/test_ABA.cpp
    test/test_T_R_out.cpp
    test/test_v_ori.cpp
    test/test_c.cpp
    test/test_p.cpp
    test/test_back.cpp
    test/test_pass3.cpp
)

target_link_libraries(ABA_test PRIVATE
    pinocchio::pinocchio
    Eigen3::Eigen #在pinocchio中使用
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_definitions(ABA_test PRIVATE
    PINOCCHIO_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)

# 加入pinocchio对比的可执行文件
# add_executable(com_with_pinocchio Com_with_pinocchio/com_with_pinocchio.cpp)

# # 链接 pinocchio 库
# target_link_libraries(com_with_pinocchio PRIVATE pinocchio::pinocchio)