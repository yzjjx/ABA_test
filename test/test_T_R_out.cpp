/**
 * @file test_T_R_out.cpp
 * @brief 该代码用于静态数组实现齐次变换矩阵、旋转矩阵、位移矩阵
 *
 *
 * @author YZJ
 * @date 2026-05-17
 * @version 0.1
 */

#include <cmath>

typedef float data_t;
const int DOF = 3;

// 为了防止命名冲突，可以加入std::cos
// 用下面void编程方式就不用这里先定义
// data_t T[DOF][4][4];     // 每个关节的 4x4 齐次变换矩阵
// data_t R[DOF][3][3];     // 每个关节的 3x3 旋转矩阵
// data_t P[DOF][3][1];     // 每个关节的 3x1 位移矩阵

// 利用return只能返回一个值，因此使用void
void cal_R(
    const data_t alpha[DOF],
    const data_t a[DOF],
    const data_t d[DOF],
    const data_t q[DOF],
    data_t T[DOF][4][4],
    data_t R[DOF][3][3],
    data_t R_Trans[DOF][3][3],
    data_t P[DOF][3])
{
    for(int i = 0;i<DOF;i++)
    {
        // 齐次变换矩阵
        T[i][0][0] = cos(q[i]);
        T[i][0][1] = -sin(q[i]);
        T[i][0][2] = 0;
        T[i][0][3] = a[i];

        T[i][1][0] = sin(q[i])*cos(alpha[i]);
        T[i][1][1] = cos(q[i])*cos(alpha[i]);
        T[i][1][2] = -sin(alpha[i]);
        T[i][1][3] = -sin(alpha[i])*d[i];

        T[i][2][0] = sin(q[i])*sin(alpha[i]);
        T[i][2][1] = cos(q[i])*sin(alpha[i]);
        T[i][2][2] = cos(alpha[i]);
        T[i][2][3] = cos(alpha[i])*d[i];

        T[i][3][0] = 0;
        T[i][3][1] = 0;
        T[i][3][2] = 0;
        T[i][3][3] = 1;

        // 旋转矩阵
        R[i][0][0] = T[i][0][0];
        R[i][0][1] = T[i][0][1];
        R[i][0][2] = T[i][0][2];

        R[i][1][0] = T[i][1][0];
        R[i][1][1] = T[i][1][1];
        R[i][1][2] = T[i][1][2];

        R[i][2][0] = T[i][2][0];
        R[i][2][1] = T[i][2][1];
        R[i][2][2] = T[i][2][2];

        // 旋转矩阵转置
        R_Trans[i][0][0] = T[i][0][0];
        R_Trans[i][0][1] = T[i][1][0];
        R_Trans[i][0][2] = T[i][2][0];

        R_Trans[i][1][0] = T[i][0][1];
        R_Trans[i][1][1] = T[i][1][1];
        R_Trans[i][1][2] = T[i][2][1];

        R_Trans[i][2][0] = T[i][0][2];
        R_Trans[i][2][1] = T[i][1][2];
        R_Trans[i][2][2] = T[i][2][2];

        // 位置矩阵
        P[i][0] = T[i][0][3];
        P[i][1] = T[i][1][3];
        P[i][2] = T[i][2][3];
    }

}