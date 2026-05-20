#ifndef TEST_T_R_OUT
#define TEST_T_R_OUT

#include <cmath>

typedef float data_t;
const int DOF = 3;

void cal_R(
    const data_t alpha[DOF],
    const data_t a[DOF],
    const data_t d[DOF],
    const data_t q[DOF],
    data_t T[DOF][4][4],
    data_t R[DOF][3][3],
    data_t R_Trans[DOF][3][3],
    data_t P[DOF][3]);

#endif