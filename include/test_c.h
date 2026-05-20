#ifndef TEST_C
#define TEST_C

#include <cmath>
#include <test_v_ori.h>

void space_cross(
    const data_t in_1[6],
    const data_t in_2[6],
    data_t out[6]
);

void c_fina(
    data_t v[DOF][6],
    data_t v_J[6][DOF],
    data_t c[DOF][6]
);

#endif