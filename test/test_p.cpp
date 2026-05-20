#include <cmath>
#include <test_c.h>

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
//     {
//         0.0380304151034530f,  0.0419599871875192f,   0.0436322529474843f,
//         0.0419599871875192f,  0.263486545009477f,   -0.00383736352324302f,
//         0.0436322529474843f, -0.00383736352324302f,  0.250878539307793f
//     },
//     {
//         0.0359185029001311f,   0.00357521327138579f, 0.000314442104644629f,
//         0.00357521327138579f,  0.00328728806178666f, 0.00270560721315391f,
//         0.000314442104644629f, 0.00270560721315391f, 0.0354951529635439f
//     },
//     {
//         0.0231030453535813f,    -2.36395069266735e-06f, -2.48326590911455e-06f,
//        -2.36395069266735e-06f,   0.0220785414360184f,    -0.00290398198945597f,
//        -2.48326590911455e-06f,  -0.00290398198945597f,   0.00301282626685543f
//     },
//     {
//         0.00568553742802938f,   1.20535594054042e-05f, -4.97420578495685e-05f,
//         1.20535594054042e-05f,  0.00444010179654207f,  0.000717862262034322f,
//        -4.97420578495685e-05f,  0.000717862262034322f, 0.00277209865650963f
//     },
//     {
//         0.00227249981806468f,  -1.38365624835237e-06f,  2.97586698701961e-06f,
//        -1.38365624835237e-06f,  0.00227800923976532f,   3.82651246364623e-06f,
//         2.97586698701961e-06f,  3.82651246364623e-06f,  0.000447957334568561f
//     }
// };

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
        corr_1_1[i][0][2] = c_of_mass_cross[i][0][0]*c_of_mass_cross[i][0][0] + c_of_mass_cross[i][0][1]*c_of_mass_cross[i][0][1] + c_of_mass_cross[i][0][2]*c_of_mass_cross[i][2][2];

        corr_1_1[i][1][0] = c_of_mass_cross[i][1][0]*c_of_mass_cross[i][0][0] + c_of_mass_cross[i][1][1]*c_of_mass_cross[i][0][1] + c_of_mass_cross[i][1][2]*c_of_mass_cross[i][0][2];
        corr_1_1[i][1][1] = c_of_mass_cross[i][1][0]*c_of_mass_cross[i][1][0] + c_of_mass_cross[i][1][1]*c_of_mass_cross[i][1][1] + c_of_mass_cross[i][1][2]*c_of_mass_cross[i][1][2];
        corr_1_1[i][1][2] = c_of_mass_cross[i][1][0]*c_of_mass_cross[i][0][0] + c_of_mass_cross[i][1][1]*c_of_mass_cross[i][0][1] + c_of_mass_cross[i][1][2]*c_of_mass_cross[i][2][2];

        corr_1_1[i][2][0] = c_of_mass_cross[i][2][0]*c_of_mass_cross[i][0][0] + c_of_mass_cross[i][2][1]*c_of_mass_cross[i][0][1] + c_of_mass_cross[i][2][2]*c_of_mass_cross[i][0][2];
        corr_1_1[i][2][1] = c_of_mass_cross[i][2][0]*c_of_mass_cross[i][1][0] + c_of_mass_cross[i][2][1]*c_of_mass_cross[i][1][1] + c_of_mass_cross[i][2][2]*c_of_mass_cross[i][1][2];
        corr_1_1[i][2][2] = c_of_mass_cross[i][2][0]*c_of_mass_cross[i][0][0] + c_of_mass_cross[i][2][1]*c_of_mass_cross[i][0][1] + c_of_mass_cross[i][2][2]*c_of_mass_cross[i][2][2];


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

// 空间动量计算
