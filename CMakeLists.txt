cmake_minimum_required(VERSION 3.10)
project(ABA_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")

find_package(pinocchio REQUIRED)

add_executable(ABA_test 
    test/test_ABA.cpp
    test/test_T_R_out.cpp
    test/test_v_ori.cpp
    test/test_c.cpp
    test/test_p.cpp
)

target_link_libraries(ABA_test PRIVATE
    pinocchio::pinocchio
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_definitions(ABA_test PRIVATE
    PINOCCHIO_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)