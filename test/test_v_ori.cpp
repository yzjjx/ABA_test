#include <cmath>
#include <test_T_R_out.h>

// 因为在test_T_R_out已经定义，下面不再定义
// typedef float data_t;
// const int DOF = 3;

void cal_v_J(
    const data_t dq[DOF],
    data_t v_J[6][DOF])
{
    data_t S[6] = {{0},{0},{1},{0},{0},{0}};

    for(int i = 0;i<DOF;i++)
    {
        v_J[0][i] = S[0]*dq[i];
        v_J[1][i] = S[1]*dq[i];
        v_J[2][i] = S[2]*dq[i];
        v_J[3][i] = S[3]*dq[i];
        v_J[4][i] = S[4]*dq[i];
        v_J[5][i] = S[5]*dq[i];
    }
}

void v_fina(
    const data_t alpha[DOF],
    const data_t a[DOF],
    const data_t d[DOF],
    const data_t q[DOF],
    const data_t dq[DOF],
    data_t X_lam[DOF][6][6],
    data_t v[DOF][6]
)
{
    // 定义X_lam的右下角矩阵
    data_t X_lam_22[DOF][3][3];

    data_t v_J_local[6][DOF];
    cal_v_J(dq, v_J_local);

    // 定义输出数组
    data_t T[DOF][4][4];
    data_t R[DOF][3][3];
    data_t R_Trans[DOF][3][3];
    data_t P[DOF][3];
    cal_R(alpha, a, d, q, T, R, R_Trans, P);

    data_t p_cross[DOF][3][3];
    for(int i = 0;i < DOF;i++)
    {
        p_cross[i][0][0] = 0;
        p_cross[i][0][1] = -P[i][2];
        p_cross[i][0][2] = P[i][1];

        p_cross[i][1][0] = P[i][2];
        p_cross[i][1][1] = 0;
        p_cross[i][1][2] = -P[i][0];

        p_cross[i][2][0] = -P[i][1];
        p_cross[i][2][1] = P[i][0];
        p_cross[i][2][2] = 0;
    }

    for (int i = 0; i < DOF; i++)
    {
        X_lam_22[i][0][0] = -(R_Trans[i][0][0]*p_cross[i][0][0]+
                              R_Trans[i][0][1]*p_cross[i][1][0]+
                              R_Trans[i][0][2]*p_cross[i][2][0]);
        X_lam_22[i][0][1] = -(R_Trans[i][0][0]*p_cross[i][0][1]+
                              R_Trans[i][0][1]*p_cross[i][1][1]+
                              R_Trans[i][0][2]*p_cross[i][2][1]);
        X_lam_22[i][0][2] = -(R_Trans[i][0][0]*p_cross[i][0][2]+
                              R_Trans[i][0][1]*p_cross[i][1][2]+
                              R_Trans[i][0][2]*p_cross[i][2][2]);

        X_lam_22[i][1][0] = -(R_Trans[i][1][0]*p_cross[i][0][0]+
                              R_Trans[i][1][1]*p_cross[i][1][0]+
                              R_Trans[i][1][2]*p_cross[i][2][0]);
        X_lam_22[i][1][1] = -(R_Trans[i][1][0]*p_cross[i][0][1]+
                              R_Trans[i][1][1]*p_cross[i][1][1]+
                              R_Trans[i][1][2]*p_cross[i][2][1]);
        X_lam_22[i][1][2] = -(R_Trans[i][1][0]*p_cross[i][0][2]+
                              R_Trans[i][1][1]*p_cross[i][1][2]+
                              R_Trans[i][1][2]*p_cross[i][2][2]);

        X_lam_22[i][2][0] = -(R_Trans[i][2][0]*p_cross[i][0][0]+
                              R_Trans[i][2][1]*p_cross[i][1][0]+
                              R_Trans[i][2][2]*p_cross[i][2][0]);
        X_lam_22[i][2][1] = -(R_Trans[i][2][0]*p_cross[i][0][1]+
                              R_Trans[i][2][1]*p_cross[i][1][1]+
                              R_Trans[i][2][2]*p_cross[i][2][1]);
        X_lam_22[i][2][2] = -(R_Trans[i][2][0]*p_cross[i][0][2]+
                              R_Trans[i][2][1]*p_cross[i][1][2]+
                              R_Trans[i][2][2]*p_cross[i][2][2]);
    }
    
    for (int i = 0; i < DOF; i++)
    {
        X_lam[i][0][0] = R_Trans[i][0][0];
        X_lam[i][0][1] = R_Trans[i][0][1];
        X_lam[i][0][2] = R_Trans[i][0][2];
        X_lam[i][0][3] = 0;
        X_lam[i][0][4] = 0;
        X_lam[i][0][5] = 0;

        X_lam[i][1][0] = R_Trans[i][1][0];
        X_lam[i][1][1] = R_Trans[i][1][1];
        X_lam[i][1][2] = R_Trans[i][1][2];
        X_lam[i][1][3] = 0;
        X_lam[i][1][4] = 0;
        X_lam[i][1][5] = 0;

        X_lam[i][2][0] = R_Trans[i][2][0];
        X_lam[i][2][1] = R_Trans[i][2][1];
        X_lam[i][2][2] = R_Trans[i][2][2];
        X_lam[i][2][3] = 0;
        X_lam[i][2][4] = 0;
        X_lam[i][2][5] = 0;

        X_lam[i][3][0] = X_lam_22[i][0][0];
        X_lam[i][3][1] = X_lam_22[i][0][1];
        X_lam[i][3][2] = X_lam_22[i][0][2];
        X_lam[i][3][3] = R_Trans[i][0][0];
        X_lam[i][3][4] = R_Trans[i][0][1];
        X_lam[i][3][5] = R_Trans[i][0][2];

        X_lam[i][4][0] = X_lam_22[i][1][0];
        X_lam[i][4][1] = X_lam_22[i][1][1];
        X_lam[i][4][2] = X_lam_22[i][1][2];
        X_lam[i][4][3] = R_Trans[i][1][0];
        X_lam[i][4][4] = R_Trans[i][1][1];
        X_lam[i][4][5] = R_Trans[i][1][2];

        X_lam[i][5][0] = X_lam_22[i][2][0];
        X_lam[i][5][1] = X_lam_22[i][2][1];
        X_lam[i][5][2] = X_lam_22[i][2][2];
        X_lam[i][5][3] = R_Trans[i][2][0];
        X_lam[i][5][4] = R_Trans[i][2][1];
        X_lam[i][5][5] = R_Trans[i][2][2];
    }
      
    // 速度前向递推计算
    for(int i = 0; i < DOF; i++)
    {
        if (i == 0) 
        {
            // 对于第一个连杆 (i=0)，父节点是基座(速度为0)。
            // 因此 v0 = X * v_base + v_J0 = v_J0
            v[0][0] = v_J_local[0][0]; 
            v[0][1] = v_J_local[1][0]; 
            v[0][2] = v_J_local[2][0]; 
            v[0][3] = v_J_local[3][0]; 
            v[0][4] = v_J_local[4][0]; 
            v[0][5] = v_J_local[5][0]; 
        } 
        else 
        {
            // 对于后续连杆，使用前向递推: v_i = X_i * v_{i-1} + v_Ji
            v[i][0] = X_lam[i][0][0]*v[i-1][0] +
                      X_lam[i][0][1]*v[i-1][1] +
                      X_lam[i][0][2]*v[i-1][2] +
                      X_lam[i][0][3]*v[i-1][3] +
                      X_lam[i][0][4]*v[i-1][4] +
                      X_lam[i][0][5]*v[i-1][5] +
                      v_J_local[0][i]; 

            v[i][1] = X_lam[i][1][0]*v[i-1][0] +
                      X_lam[i][1][1]*v[i-1][1] +
                      X_lam[i][1][2]*v[i-1][2] +
                      X_lam[i][1][3]*v[i-1][3] +
                      X_lam[i][1][4]*v[i-1][4] +
                      X_lam[i][1][5]*v[i-1][5] +
                      v_J_local[1][i]; 

            v[i][2] = X_lam[i][2][0]*v[i-1][0] +
                      X_lam[i][2][1]*v[i-1][1] +
                      X_lam[i][2][2]*v[i-1][2] +
                      X_lam[i][2][3]*v[i-1][3] +
                      X_lam[i][2][4]*v[i-1][4] +
                      X_lam[i][2][5]*v[i-1][5] +
                      v_J_local[2][i]; 

            v[i][3] = X_lam[i][3][0]*v[i-1][0] +
                      X_lam[i][3][1]*v[i-1][1] +
                      X_lam[i][3][2]*v[i-1][2] +
                      X_lam[i][3][3]*v[i-1][3] +
                      X_lam[i][3][4]*v[i-1][4] +
                      X_lam[i][3][5]*v[i-1][5] +
                      v_J_local[3][i]; 

            v[i][4] = X_lam[i][4][0]*v[i-1][0] +
                      X_lam[i][4][1]*v[i-1][1] +
                      X_lam[i][4][2]*v[i-1][2] +
                      X_lam[i][4][3]*v[i-1][3] +
                      X_lam[i][4][4]*v[i-1][4] +
                      X_lam[i][4][5]*v[i-1][5] +
                      v_J_local[4][i]; 

            v[i][5] = X_lam[i][5][0]*v[i-1][0] +
                      X_lam[i][5][1]*v[i-1][1] +
                      X_lam[i][5][2]*v[i-1][2] +
                      X_lam[i][5][3]*v[i-1][3] +
                      X_lam[i][5][4]*v[i-1][4] +
                      X_lam[i][5][5]*v[i-1][5] +
                      v_J_local[5][i]; 
        }
    }
}