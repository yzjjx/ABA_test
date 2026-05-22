#include <iostream>
#include <iomanip>
#include <test_ABA_final.h>

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

    data_t tau[DOF] = {
        0,
        0,
        0
    };
    data_t ddq[DOF];

    ABA(alpha,a,d,q,dq,I,mass,c_of_mass,tau,ddq);

    for(int i=0;i<DOF;i++)
    {
        std::cout << "Joint " << i + 1 << std::endl;
        std::cout << "ddq[" << i << "] =" << std::endl;
        std::cout << std::setw(12) << ddq[i]<< " ";
        std::cout << std::endl;
    }

}