/**
 * @file test_ABA_final.cpp
 * @brief 用于串联ABA代码的所有计算部分
 *
 *
 * @author YZJ
 * @date 2026-05-22
 * @version t0.1
 */

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
)
{
    // 定义输出数组
    data_t T[DOF][4][4];
    data_t R[DOF][3][3];
    data_t R_Trans[DOF][3][3];
    data_t P[DOF][3];

    data_t v_J[6][DOF];

    data_t X_lam[DOF][6][6];
    data_t v[DOF][6];

    data_t c[DOF][6];

    data_t I_spa[DOF][6][6];
    data_t h[DOF][6];
    data_t p[DOF][6];

    data_t I_A[DOF][6][6];
    data_t p_A[DOF][6];
    data_t U[DOF][6];
    data_t D[DOF];
    data_t u[DOF];
    data_t I_a[DOF][6][6];
    data_t p_a[DOF][6];

    data_t a_s[DOF][6];
    data_t a_a[DOF][6];

    // 调用函数
    cal_R(alpha, a, d, q, T, R, R_Trans, P);

    // 计算v_J
    cal_v_J(dq,v_J);

    // 计算v
    v_fina(alpha,a,d,q,dq,X_lam,v);

    // 计算c
    c_fina(v,v_J,c);

    // 计算h
    I_space(mass,c_of_mass,I,I_spa);
    h_fina(alpha,a,d,q,dq,X_lam,mass,c_of_mass,I,I_spa,v,h);
    p_fina(v,h,p);

    // 逆推
    back_pass(I_spa,p,tau,c,X_lam,I_A,p_A,U,D,u,I_a,p_a);

    // 正推
    tip_pass3(X_lam,c,D,u,U,a_s,a_a,ddq);
}