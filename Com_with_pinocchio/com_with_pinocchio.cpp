// pinocchio三轴机器人计算
#include <iostream>
#include <iomanip>
#include <cmath>

// 引入 Pinocchio 头文件
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/aba.hpp"

int main()
{
    // 1. 创建机器人模型
    pinocchio::Model model;

    // 假设未定义重力（根据你的原始代码无重力参数推断）
    // 如果你的算法包含了标准重力补偿，请注释掉这一行，Pinocchio默认是 (0, 0, -9.81)
    // model.gravity.setZero(); 

    // 2. MDH参数定义与关节添加
    // 关节 1 (从基座开始)
    // alpha = 0, a = 0, d = 0.3
    double alpha1 = 0.0;
    double a1 = 0.0;
    double d1 = 0.3;
    pinocchio::SE3 placement1;
    placement1.translation() << a1, -d1 * std::sin(alpha1), d1 * std::cos(alpha1);
    placement1.rotation() = Eigen::AngleAxisd(alpha1, Eigen::Vector3d::UnitX()).toRotationMatrix();
    
    auto joint1_id = model.addJoint(0, pinocchio::JointModelRZ(), placement1, "Joint1");

    // 配置连杆 1 的惯性参数 (质量, 质心位置, 质心处的惯性张量)
    Eigen::Matrix3d I1;
    I1 << 0.022745,   0,          0,
          0,          0.022745,   0,
          0,          0,          0.00175;
    pinocchio::Inertia inertia1(2.916, Eigen::Vector3d(0, 0, -0.15), I1);
    model.appendBodyToJoint(joint1_id, inertia1, pinocchio::SE3::Identity());

    // 关节 2
    // alpha = -90 deg, a = 0.05, d = 0
    double alpha2 = -90.0 * M_PI / 180.0;
    double a2 = 0.05;
    double d2 = 0.0;
    pinocchio::SE3 placement2;
    placement2.translation() << a2, -d2 * std::sin(alpha2), d2 * std::cos(alpha2);
    placement2.rotation() = Eigen::AngleAxisd(alpha2, Eigen::Vector3d::UnitX()).toRotationMatrix();

    auto joint2_id = model.addJoint(joint1_id, pinocchio::JointModelRZ(), placement2, "Joint2");

    Eigen::Matrix3d I2;
    I2 << 0.000054, 0,          0,
          0,        0.012366,   0,
          0,        0,          0.012488;
    pinocchio::Inertia inertia2(1.620, Eigen::Vector3d(0.15, 0, 0), I2);
    model.appendBodyToJoint(joint2_id, inertia2, pinocchio::SE3::Identity());

    // 关节 3
    // alpha = 90 deg, a = 0.3, d = 0
    double alpha3 = 90.0 * M_PI / 180.0;
    double a3 = 0.3;
    double d3 = 0.0;
    pinocchio::SE3 placement3;
    placement3.translation() << a3, -d3 * std::sin(alpha3), d3 * std::cos(alpha3);
    placement3.rotation() = Eigen::AngleAxisd(alpha3, Eigen::Vector3d::UnitX()).toRotationMatrix();

    auto joint3_id = model.addJoint(joint2_id, pinocchio::JointModelRZ(), placement3, "Joint3");

    Eigen::Matrix3d I3;
    I3 << 0.000222, 0,          0,
          0,        0.005018,   0,
          0,        0,          0.005048;
    pinocchio::Inertia inertia3(0.945, Eigen::Vector3d(0.125, 0, 0), I3);
    model.appendBodyToJoint(joint3_id, inertia3, pinocchio::SE3::Identity());


    // 3. 定义状态向量：位置 q, 速度 v, 力矩 tau
    Eigen::VectorXd q(model.nq);
    q << 30.0 * M_PI / 180.0,
        -20.0 * M_PI / 180.0,
         40.0 * M_PI / 180.0;

    Eigen::VectorXd v(model.nv);
    v << 0.8,
        -0.5,
         0.6;

    Eigen::VectorXd tau(model.nv);
    tau << 0.0, 
           0.0, 
           0.0;

    // 4. 创建 Data 结构并运行 ABA (Articulated Body Algorithm)
    pinocchio::Data data(model);
    pinocchio::aba(model, data, q, v, tau);

    // 5. 打印结果，验证 ddq
    std::cout << "============= Pinocchio ABA Result =============" << std::endl;
    std::cout << "q (rad)  : " << q.transpose() << std::endl;
    std::cout << "dq (rad/s): " << v.transpose() << std::endl;
    std::cout << "tau (N.m) : " << tau.transpose() << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
    
    // 提取计算出的关节加速度
    std::cout << "ddq [0]  = " << std::setw(12) << data.ddq[0] << " (rad/s^2)" << std::endl;
    std::cout << "ddq [1]  = " << std::setw(12) << data.ddq[1] << " (rad/s^2)" << std::endl;
    std::cout << "ddq [2]  = " << std::setw(12) << data.ddq[2] << " (rad/s^2)" << std::endl;

    return 0;
}