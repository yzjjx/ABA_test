#include <test_T_R_out.h>
#include <iostream>
#include <iomanip>
#include <test_v_ori.h>
#include <test_c.h>
#include <test_p.h>
// iomapip包含setw(12)，可以保持输出为12个字符宽度，可以对齐输出

const data_t PI = 3.14159265358979323846f;

int main()
{
    // 定义 MDH 参数，单位：m 和 rad
    data_t alpha[DOF] = {
        0.0f * PI / 180.0f,
        -90.0f * PI / 180.0f,
        90.0f * PI / 180.0f
    };

    data_t a[DOF] = {
        0.0f,
        0.05f,
        0.3f
    };

    data_t d[DOF] = {
        0.3f,
        0.0f,
        0.0f
    };

    // 定义关节角 q，单位：rad
    data_t q[DOF] = {
        30.0f * PI / 180.0f,
        -20.0f * PI / 180.0f,
        40.0f * PI / 180.0f
    };

    // 定义关节角加速度 dq，单位：rad
    data_t dq[DOF] = {
        0.8,
        -0.5,
        0.6
    };

    // 惯性张量（按 3x3 行优先展开）
    data_t I[DOF][9] = {
        {
            0.022745,   0,          0,
            0,          0.022745,   0,
            0,          0,          0.00175
        },
        {
            0.000054,  0,          0,
            0,         0.012366,   0,
            0,         0,          0.012488
        },
        {
            0.000222,   0,          0,
            0,          0.005018,   0,
            0,          0,          0.005048
        }
    };
    data_t mass[DOF] = {
        {2.916},
        {1.620},
        {0.945}
    };
    data_t c_of_mass[DOF][3] = {
        {0,0,-0.15},
        {0.15,0,0},
        {0.125,0,0}
    };

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


    // 输出结果
    for (int i = 0; i < DOF; i++)
    {
        std::cout << "==============================" << std::endl;
        std::cout << "Joint " << i + 1 << std::endl;
 
        std::cout << "T[" << i << "] =" << std::endl;
        for (int r = 0; r < 4; r++)
        {
            for (int c = 0; c < 4; c++)
            {
                std::cout << std::setw(12) << T[i][r][c] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << std::endl;

        std::cout << "R[" << i << "] =" << std::endl;
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                std::cout << std::setw(12) << R[i][r][c] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << std::endl;

        std::cout << "R_Trans[" << i << "] =" << std::endl;
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                std::cout << std::setw(12) << R_Trans[i][r][c] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "v_J[" << i << "] =" << std::endl;
        for (int r = 0; r < 6; r++)
        {
            std::cout << std::setw(12) << v_J[r][i] << " ";
            std::cout << std::endl;
        }

        std::cout << "v[" << i << "] =" << std::endl;
        for (int r = 0; r < 6; r++)
        {
            std::cout << std::setw(12) << v[i][r] << " ";
            std::cout << std::endl;
        }

        std::cout << "c[" << i << "] =" << std::endl;
        for (int r = 0; r < 6; r++)
        {
            std::cout << std::setw(12) << c[i][r] << " ";
            std::cout << std::endl;
        }

        std::cout << "I_spa[" << i << "] =" << std::endl;
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                std::cout << std::setw(12) << I_spa[i][r][c] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "h[" << i << "] =" << std::endl;
        for (int r = 0; r < 6; r++)
        {
            std::cout << std::setw(12) << h[i][r] << " ";
            std::cout << std::endl;
        }


        std::cout << "p[" << i << "] =" << std::endl;
        for (int r = 0; r < 6; r++)
        {
            std::cout << std::setw(12) << p[i][r] << " ";
            std::cout << std::endl;
        }

        std::cout << std::endl;

        std::cout << "P[" << i << "] =" << std::endl;
        std::cout << std::setw(12) << P[i][0] << std::endl;
        std::cout << std::setw(12) << P[i][1] << std::endl;
        std::cout << std::setw(12) << P[i][2] << std::endl;

        std::cout << std::endl;
    }
}

// int main()
// {
//     const data_t PI = 3.14159265358979323846f;

//     // 定义 MDH 参数
//     data_t alpha[DOF] = {
//         0.0f,
//         PI / 2.0f,
//         0.0f
//     };

//     data_t a[DOF] = {
//         0.3f,
//         0.2f,
//         0.1f
//     };

//     data_t d[DOF] = {
//         0.1f,
//         0.0f,
//         0.0f
//     };

//     // 关节角，单位是弧度
//     data_t q[DOF] = {
//         30.0f * PI / 180.0f,
//         45.0f * PI / 180.0f,
//         60.0f * PI / 180.0f
//     };

//     // 定义输出数组
//     data_t T[DOF][4][4];
//     data_t R[DOF][3][3];
//     data_t R_Trans[DOF][3][3];
//     data_t P[DOF][3];

//     // 调用函数
//     cal_R(alpha, a, d, q, T, R, R_Trans, P);

//     // 设置输出格式
//     std::cout << std::fixed << std::setprecision(6);

//     // 输出结果
//     for (int i = 0; i < DOF; i++)
//     {
//         std::cout << "==============================" << std::endl;
//         std::cout << "Joint " << i + 1 << std::endl;

//         std::cout << "T[" << i << "] =" << std::endl;
//         for (int r = 0; r < 4; r++)
//         {
//             for (int c = 0; c < 4; c++)
//             {
//                 std::cout << std::setw(12) << T[i][r][c] << " ";
//             }
//             std::cout << std::endl;
//         }

//         std::cout << std::endl;

//         std::cout << "R[" << i << "] =" << std::endl;
//         for (int r = 0; r < 3; r++)
//         {
//             for (int c = 0; c < 3; c++)
//             {
//                 std::cout << std::setw(12) << R[i][r][c] << " ";
//             }
//             std::cout << std::endl;
//         }

//         std::cout << std::endl;

//         std::cout << "R_Trans[" << i << "] =" << std::endl;
//         for (int r = 0; r < 3; r++)
//         {
//             for (int c = 0; c < 3; c++)
//             {
//                 std::cout << std::setw(12) << R_Trans[i][r][c] << " ";
//             }
//             std::cout << std::endl;
//         }

//         std::cout << std::endl;

//         std::cout << "P[" << i << "] =" << std::endl;
//         std::cout << std::setw(12) << P[i][0] << std::endl;
//         std::cout << std::setw(12) << P[i][1] << std::endl;
//         std::cout << std::setw(12) << P[i][2] << std::endl;

//         std::cout << std::endl;
//     }

//     return 0;
// }