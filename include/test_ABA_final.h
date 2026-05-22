#ifndef TEST_ABA_FINAL
#define TEST_ABA_FINAL

#include <test_T_R_out.h>
#include <test_v_ori.h>
#include <test_c.h>
#include <test_p.h>
#include <test_back.h>
#include <test_pass3.h>

void ABA(
    const data_t alpha[DOF],
    const data_t a[DOF],
    const data_t d[DOF],
    const data_t q[DOF],
    const data_t dq[DOF],
    const data_t I[DOF][9],
    const data_t mass[DOF],
    const data_t c_of_mass[DOF][3],
    const data_t tau[DOF],
    data_t ddq[DOF]
);

#endif