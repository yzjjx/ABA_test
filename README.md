该代码用于静态数组C++实现ABA机器人正动力学算法，用于HLS综合与FPGA并行加速计算
test文件：
    test/test_ABA.cpp：三轴机器人的ABA测试输出计算文件，目前计算结果为正确的计算结果
    test/test_T_R_out.cpp：用于根据DH表生成齐次变换矩阵、旋转矩阵、位移矩阵（测试文件）
    test/test_v_ori.cpp：用于实现速度的前向递推计算，主要根据为书本的Pass1部分的公式4
    test/test_c.cpp:用于实现xxxx计算，主要根据为书本的Pass1部分的公式5
    
    test/test_back.cpp：
