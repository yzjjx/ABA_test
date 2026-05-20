#include <cmath>
#include <test_v_ori.h>

// 因为在test_T_R_out已经定义，下面不再定义
// typedef float data_t;
// const int DOF = 3;

// 定义静态数组空间向量叉乘方式
void space_cross(
    const data_t in_1[6],
    const data_t in_2[6],
    data_t out[6]
)
{
    out[0] = in_1[1]*in_2[2]-in_1[2]*in_2[1];
    out[1] = in_1[2]*in_2[0]-in_1[0]*in_2[2];
    out[2] = in_1[0]*in_2[1]-in_1[1]*in_2[0];

    out[3] = in_1[1]*in_2[5]-in_1[2]*in_2[4]+ 
             in_1[4]*in_2[2]-in_1[5]*in_2[1];
    out[4] = in_1[2]*in_2[3]-in_1[0]*in_2[5]+ 
             in_1[5]*in_2[0]-in_1[3]*in_2[2];
    out[5] = in_1[0]*in_2[4]-in_1[1]*in_2[3]+ 
             in_1[3]*in_2[1]-in_1[4]*in_2[0];

}

// 因为是转动副，所以S圈i为0,因此c的计算只有后面的叉乘，为6*1矩阵叉乘
void c_fina(
    data_t v[DOF][6],
    data_t v_J[6][DOF],
    data_t c[DOF][6]
)
{   
    for(int i = 0;i < DOF;i++)
    {
        // 1. 提取 v_J 的第 i 列（第 i 个关节的 6D 速度）
        data_t current_v_J[6];
        current_v_J[0] = v_J[0][i];
        current_v_J[1] = v_J[1][i];
        current_v_J[2] = v_J[2][i];
        current_v_J[3] = v_J[3][i];
        current_v_J[4] = v_J[4][i];
        current_v_J[5] = v_J[5][i];

        space_cross(v[i], current_v_J, c[i]);
    }
}