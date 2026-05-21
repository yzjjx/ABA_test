#ifndef TEST_PASS3
#define TEST_PASS3

#include <cmath>
#include <test_v_ori.h>
#include <test_p.h>

void tip_pass3(
    const data_t X_lam[DOF][6][6],
    const data_t c[DOF][6],
    const data_t D[DOF],
    const data_t u[DOF],
    const data_t U[DOF][6],
    data_t a_s[DOF][6],
    data_t a_a[DOF][6],
    data_t ddq[DOF]
);

#endif