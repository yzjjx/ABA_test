#ifndef TEST_P
#define TEST_P

#include <cmath>
#include <test_c.h>
#include <test_v_ori.h>

void I_space(
    data_t mass[DOF],
    data_t c_of_mass[DOF][3],
    data_t I[DOF][9],//行展开矩阵
    data_t I_spa[DOF][6][6]
);

void h_fina(
    const data_t alpha[DOF],
    const data_t a[DOF],
    const data_t d[DOF],
    const data_t q[DOF],
    const data_t dq[DOF],
    data_t X_lam[DOF][6][6],

    data_t mass[DOF],
    data_t c_of_mass[DOF][3],
    data_t I[DOF][9],//行展开矩阵
    data_t I_spa[DOF][6][6],
    data_t v[DOF][6],
    data_t h[DOF][6]
);

#endif