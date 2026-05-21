#include <cmath>
#include <test_c.h>
#include <test_v_ori.h>

// 因为在test_T_R_out已经定义，下面不再定义
// typedef float data_t;
// const int DOF = 3;

// 外力为0,后面项为0，仅计算前面的空间叉乘

// // 惯性张量（按 3x3 行优先展开）
// static const data_t I[DOF][9] = {
//     {
//         0.058267910813839f,      -3.715826953316560e-07f, -1.05949175179955e-06f,
//        -3.715826953316560e-07f,  0.0576103483714700f,     -0.00456914016524626f,
//        -1.05949175179955e-06f,  -0.00456914016524626f,    0.00607968319952077f
//     },

// 计算空间惯量矩阵
void I_space(
    data_t mass[DOF],
    data_t c_of_mass[DOF][3],
    data_t I[DOF][9],//行展开矩阵
    data_t I_spa[DOF][6][6]
)
{
    // 计算质心叉乘矩阵
    data_t c_of_mass_cross[DOF][3][3];
    for(int i = 0;i < DOF;i++)
    {
        c_of_mass_cross[i][0][0] = 0;
        c_of_mass_cross[i][0][1] = -c_of_mass[i][2];
        c_of_mass_cross[i][0][2] = c_of_mass[i][1];

        c_of_mass_cross[i][1][0] = c_of_mass[i][2];
        c_of_mass_cross[i][1][1] = 0;
        c_of_mass_cross[i][1][2] = -c_of_mass[i][0];
    
        c_of_mass_cross[i][2][0] = -c_of_mass[i][1];
        c_of_mass_cross[i][2][1] = c_of_mass[i][0];
        c_of_mass_cross[i][2][2] = 0;
    }

    // 计算平行轴修正项（空间惯量矩阵左上角部分项）
    data_t corr_1_1[DOF][3][3];
    for(int i = 0;i < DOF;i++)
    {
        corr_1_1[i][0][0] = c_of_mass_cross[i][0][0]*c_of_mass_cross[i][0][0] + c_of_mass_cross[i][0][1]*c_of_mass_cross[i][0][1] + c_of_mass_cross[i][0][2]*c_of_mass_cross[i][0][2];
        corr_1_1[i][0][1] = c_of_mass_cross[i][0][0]*c_of_mass_cross[i][1][0] + c_of_mass_cross[i][0][1]*c_of_mass_cross[i][1][1] + c_of_mass_cross[i][0][2]*c_of_mass_cross[i][1][2];
        corr_1_1[i][0][2] = c_of_mass_cross[i][0][0]*c_of_mass_cross[i][2][0] + c_of_mass_cross[i][0][1]*c_of_mass_cross[i][2][1] + c_of_mass_cross[i][0][2]*c_of_mass_cross[i][2][2];

        corr_1_1[i][1][0] = c_of_mass_cross[i][1][0]*c_of_mass_cross[i][0][0] + c_of_mass_cross[i][1][1]*c_of_mass_cross[i][0][1] + c_of_mass_cross[i][1][2]*c_of_mass_cross[i][0][2];
        corr_1_1[i][1][1] = c_of_mass_cross[i][1][0]*c_of_mass_cross[i][1][0] + c_of_mass_cross[i][1][1]*c_of_mass_cross[i][1][1] + c_of_mass_cross[i][1][2]*c_of_mass_cross[i][1][2];
        corr_1_1[i][1][2] = c_of_mass_cross[i][1][0]*c_of_mass_cross[i][2][0] + c_of_mass_cross[i][1][1]*c_of_mass_cross[i][2][1] + c_of_mass_cross[i][1][2]*c_of_mass_cross[i][2][2];

        corr_1_1[i][2][0] = c_of_mass_cross[i][2][0]*c_of_mass_cross[i][0][0] + c_of_mass_cross[i][2][1]*c_of_mass_cross[i][0][1] + c_of_mass_cross[i][2][2]*c_of_mass_cross[i][0][2];
        corr_1_1[i][2][1] = c_of_mass_cross[i][2][0]*c_of_mass_cross[i][1][0] + c_of_mass_cross[i][2][1]*c_of_mass_cross[i][1][1] + c_of_mass_cross[i][2][2]*c_of_mass_cross[i][1][2];
        corr_1_1[i][2][2] = c_of_mass_cross[i][2][0]*c_of_mass_cross[i][2][0] + c_of_mass_cross[i][2][1]*c_of_mass_cross[i][2][1] + c_of_mass_cross[i][2][2]*c_of_mass_cross[i][2][2];


        corr_1_1[i][0][0] = mass[i]*corr_1_1[i][0][0];
        corr_1_1[i][0][1] = mass[i]*corr_1_1[i][0][1];
        corr_1_1[i][0][2] = mass[i]*corr_1_1[i][0][2];

        corr_1_1[i][1][0] = mass[i]*corr_1_1[i][1][0];
        corr_1_1[i][1][1] = mass[i]*corr_1_1[i][1][1];
        corr_1_1[i][1][2] = mass[i]*corr_1_1[i][1][2];

        corr_1_1[i][2][0] = mass[i]*corr_1_1[i][2][0];
        corr_1_1[i][2][1] = mass[i]*corr_1_1[i][2][1];
        corr_1_1[i][2][2] = mass[i]*corr_1_1[i][2][2];
    }

    // 计算空间惯量矩阵左上角
    data_t I_1_1[DOF][3][3];
    for(int i = 0;i < DOF;i++)
    {
        I_1_1[i][0][0] = I[i][0] + corr_1_1[i][0][0];
        I_1_1[i][0][1] = I[i][1] + corr_1_1[i][0][1];
        I_1_1[i][0][2] = I[i][2] + corr_1_1[i][0][2];

        I_1_1[i][1][0] = I[i][3] + corr_1_1[i][1][0];
        I_1_1[i][1][1] = I[i][4] + corr_1_1[i][1][1];
        I_1_1[i][1][2] = I[i][5] + corr_1_1[i][1][2];

        I_1_1[i][2][0] = I[i][6] + corr_1_1[i][2][0];
        I_1_1[i][2][1] = I[i][7] + corr_1_1[i][2][1];
        I_1_1[i][2][2] = I[i][8] + corr_1_1[i][2][2];
    }

    // 计算空间惯量矩阵右上角
    data_t I_1_2[DOF][3][3];
    for(int i = 0;i<DOF;i++)
    {
        I_1_2[i][0][0] = mass[i]*c_of_mass_cross[i][0][0];
        I_1_2[i][0][1] = mass[i]*c_of_mass_cross[i][0][1];
        I_1_2[i][0][2] = mass[i]*c_of_mass_cross[i][0][2];

        I_1_2[i][1][0] = mass[i]*c_of_mass_cross[i][1][0];
        I_1_2[i][1][1] = mass[i]*c_of_mass_cross[i][1][1];
        I_1_2[i][1][2] = mass[i]*c_of_mass_cross[i][1][2];

        I_1_2[i][2][0] = mass[i]*c_of_mass_cross[i][2][0];
        I_1_2[i][2][1] = mass[i]*c_of_mass_cross[i][2][1];
        I_1_2[i][2][2] = mass[i]*c_of_mass_cross[i][2][2];
    }

    // 计算空间惯量矩阵左下角
    data_t I_2_1[DOF][3][3];
    for(int i = 0;i<DOF;i++)
    {
        I_2_1[i][0][0] = mass[i]*c_of_mass_cross[i][0][0];
        I_2_1[i][0][1] = mass[i]*c_of_mass_cross[i][1][0];
        I_2_1[i][0][2] = mass[i]*c_of_mass_cross[i][2][0];

        I_2_1[i][1][0] = mass[i]*c_of_mass_cross[i][0][1];
        I_2_1[i][1][1] = mass[i]*c_of_mass_cross[i][1][1];
        I_2_1[i][1][2] = mass[i]*c_of_mass_cross[i][2][1];

        I_2_1[i][2][0] = mass[i]*c_of_mass_cross[i][0][2];
        I_2_1[i][2][1] = mass[i]*c_of_mass_cross[i][1][2];
        I_2_1[i][2][2] = mass[i]*c_of_mass_cross[i][2][2];
    }

    // 空间惯量矩阵生成
    for(int i = 0;i<DOF;i++)
    {
        I_spa[i][0][0] = I_1_1[i][0][0];
        I_spa[i][0][1] = I_1_1[i][0][1];
        I_spa[i][0][2] = I_1_1[i][0][2];
        I_spa[i][0][3] = I_1_2[i][0][0];
        I_spa[i][0][4] = I_1_2[i][0][1];
        I_spa[i][0][5] = I_1_2[i][0][2];

        I_spa[i][1][0] = I_1_1[i][1][0];
        I_spa[i][1][1] = I_1_1[i][1][1];
        I_spa[i][1][2] = I_1_1[i][1][2];
        I_spa[i][1][3] = I_1_2[i][1][0];
        I_spa[i][1][4] = I_1_2[i][1][1];
        I_spa[i][1][5] = I_1_2[i][1][2];

        I_spa[i][2][0] = I_1_1[i][2][0];
        I_spa[i][2][1] = I_1_1[i][2][1];
        I_spa[i][2][2] = I_1_1[i][2][2];
        I_spa[i][2][3] = I_1_2[i][2][0];
        I_spa[i][2][4] = I_1_2[i][2][1];
        I_spa[i][2][5] = I_1_2[i][2][2];

        I_spa[i][3][0] = I_2_1[i][0][0];
        I_spa[i][3][1] = I_2_1[i][0][1];
        I_spa[i][3][2] = I_2_1[i][0][2];
        I_spa[i][3][3] = mass[i];
        I_spa[i][3][4] = 0;
        I_spa[i][3][5] = 0;

        I_spa[i][4][0] = I_2_1[i][1][0];
        I_spa[i][4][1] = I_2_1[i][1][1];
        I_spa[i][4][2] = I_2_1[i][1][2];
        I_spa[i][4][3] = 0;
        I_spa[i][4][4] = mass[i];
        I_spa[i][4][5] = 0;

        I_spa[i][5][0] = I_2_1[i][2][0];
        I_spa[i][5][1] = I_2_1[i][2][1];
        I_spa[i][5][2] = I_2_1[i][2][2];
        I_spa[i][5][3] = 0;
        I_spa[i][5][4] = 0;
        I_spa[i][5][5] = mass[i];
    }
    
}

// 空间动量(h)计算
// 计算空间惯量矩阵
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
)
{
    I_space(mass,c_of_mass,I,I_spa);
    v_fina(alpha,a,d,q,dq,X_lam,v);

    for (int i = 0; i < DOF; i++)
    {
        h[i][0] = I_spa[i][0][0]*v[i][0]+
                  I_spa[i][0][1]*v[i][1]+
                  I_spa[i][0][2]*v[i][2]+
                  I_spa[i][0][3]*v[i][3]+
                  I_spa[i][0][4]*v[i][4]+
                  I_spa[i][0][5]*v[i][5];

        h[i][1] = I_spa[i][1][0]*v[i][0]+
                  I_spa[i][1][1]*v[i][1]+
                  I_spa[i][1][2]*v[i][2]+
                  I_spa[i][1][3]*v[i][3]+
                  I_spa[i][1][4]*v[i][4]+
                  I_spa[i][1][5]*v[i][5];

        h[i][2] = I_spa[i][2][0]*v[i][0]+
                  I_spa[i][2][1]*v[i][1]+
                  I_spa[i][2][2]*v[i][2]+
                  I_spa[i][2][3]*v[i][3]+
                  I_spa[i][2][4]*v[i][4]+
                  I_spa[i][2][5]*v[i][5];

        h[i][3] = I_spa[i][3][0]*v[i][0]+
                  I_spa[i][3][1]*v[i][1]+
                  I_spa[i][3][2]*v[i][2]+
                  I_spa[i][3][3]*v[i][3]+
                  I_spa[i][3][4]*v[i][4]+
                  I_spa[i][3][5]*v[i][5];

        h[i][4] = I_spa[i][4][0]*v[i][0]+
                  I_spa[i][4][1]*v[i][1]+
                  I_spa[i][4][2]*v[i][2]+
                  I_spa[i][4][3]*v[i][3]+
                  I_spa[i][4][4]*v[i][4]+
                  I_spa[i][4][5]*v[i][5];

        h[i][5] = I_spa[i][5][0]*v[i][0]+
                  I_spa[i][5][1]*v[i][1]+
                  I_spa[i][5][2]*v[i][2]+
                  I_spa[i][5][3]*v[i][3]+
                  I_spa[i][5][4]*v[i][4]+
                  I_spa[i][5][5]*v[i][5];
    }
    
}

// 空间力叉乘算法
void f_space_cross(
    const data_t in_1[6],
    const data_t in_2[6],
    data_t out[6]
)
{
    out[0] = in_1[1]*in_2[2]-in_1[2]*in_2[1]+
             in_1[4]*in_2[5]-in_1[5]*in_2[4];
    out[1] = in_1[2]*in_2[0]-in_1[0]*in_2[2]+
             in_1[5]*in_2[3]-in_1[3]*in_2[5];
    out[2] = in_1[0]*in_2[1]-in_1[1]*in_2[0]+
             in_1[3]*in_2[4]-in_1[4]*in_2[3];

    out[3] = in_1[1]*in_2[5]-in_1[2]*in_2[4];
    out[4] = in_1[2]*in_2[3]-in_1[0]*in_2[5];
    out[5] = in_1[0]*in_2[4]-in_1[1]*in_2[3];
}

// 空间叉乘计算
void p_fina(
    const data_t v[DOF][6],
    const data_t h[DOF][6],
    data_t p[DOF][6]
)
{
    for(int i = 0;i<DOF;i++)
    {
        f_space_cross(v[i],h[i],p[i]);
    }  
}