#include "car_arm.h"

Eigen::Matrix<double, 19, 1> Car_arm::Solver()
{
    // 解决问题 car + left arm
    Eigen::Matrix<double, 31, 1> x;
    Eigen::Matrix<double, 31, 31> Q;
    Eigen::Matrix<double, 31, 1> C;
    Eigen::Matrix<double, 12, 31> J;
    Eigen::Matrix<double, 6, 1> V_Carbase_Leffector_star, V_Carbase_Reffector_star; // desired velocity
    Eigen::Matrix<double, 12, 1> V_Carbase_all_star;
    Eigen::Matrix<double, 17, 31> A;
    Eigen::Matrix<double, 17, 1> B;
    Eigen::Matrix<double, 31, 1> X_low, X_up;
    // tackle axis transformation

    T_World_Leffector = T_World_Carbase * T_Carbase_Base * T_Base_Leffector;
    T_World_Reffector = T_World_Carbase * T_Carbase_Base * T_Base_Reffector;

    Eigen::Matrix<double, 4, 4> T_Leffector_Eleftstar = T_World_Leffector.inverse() * T_World_Eleftstar;
    Eigen::Matrix<double, 3, 3> R_Leffector_Eleftstar = T_Leffector_Eleftstar.block(0, 0, 3, 3);

    Eigen::Matrix<double, 4, 4> T_Carbase_Leffector = T_Carbase_Base * T_Base_Leffector;
    Eigen::Matrix<double, 3, 3> R_Carbase_Leffector = T_Carbase_Leffector.block(0, 0, 3, 3);

    //
    Eigen::Matrix<double, 4, 4> T_Reffector_Erightstar = T_World_Reffector.inverse() * T_World_Erightstar;
    Eigen::Matrix<double, 3, 3> R_Reffector_Erightstar = T_Reffector_Erightstar.block(0, 0, 3, 3);

    Eigen::Matrix<double, 4, 4> T_Carbase_Reffector = T_Carbase_Base * T_Base_Reffector;
    Eigen::Matrix<double, 3, 3> R_Carbase_Reffector = T_Carbase_Reffector.block(0, 0, 3, 3);

    // ---------------------Q-----------------------------------
    Eigen::VectorXd lambda_q(19), lambda_slack(12);

    lambda_q << 2 / (dis_Carbase_Eleftstar + dis_Carbase_Erightstar), 2 / (dis_Carbase_Eleftstar + dis_Carbase_Erightstar),
        5 * ka, 5 * ka, 5 * ka,     // waist
        ka, ka, ka, ka, ka, ka, ka, // left
        ka, ka, ka, ka, ka, ka, ka; // right

    lambda_slack << 1 / dis_Leffector_Eleftstar, 1 / dis_Leffector_Eleftstar, 1 / dis_Leffector_Eleftstar,
        0.1 / dis_Leffector_Eleftstar, 0.1 / dis_Leffector_Eleftstar, 0.1 / dis_Leffector_Eleftstar,

        1 / dis_Reffector_Erightstar, 1 / dis_Reffector_Erightstar, 1 / dis_Reffector_Erightstar,
        0.1 / dis_Reffector_Erightstar, 0.1 / dis_Reffector_Erightstar, 0.1 / dis_Reffector_Erightstar;

    Eigen::VectorXd diag_value(31);
    diag_value << lambda_q, lambda_slack;
    Q = Eigen::DiagonalMatrix<double, 31>(diag_value);

    // -------------------C---------------------------------
    Matrix<double, 10, 1> Jm_Base_Leffector = Manipulability(dh_list_Base_Leffector, q_Base_Leffector);
    Matrix<double, 10, 1> Jm_Base_Reffector = Manipulability(dh_list_Base_Reffector, q_Base_Reffector);
    C(0, 0) = -k_episilon * theta_Carbase_Leffector - k_episilon * theta_Carbase_Reffector;
    C.block(2, 0, 3, 1) = -Jm_Base_Leffector.block(0, 0, 3, 1) - Jm_Base_Reffector.block(0, 0, 3, 1);
    C.block(5, 0, 7, 1) = -Jm_Base_Leffector.block(3, 0, 7, 1);
    C.block(12, 0, 7, 1) = -Jm_Base_Reffector.block(3, 0, 7, 1);
    C.block(19, 0, 12, 1) = Eigen::Matrix<double, 12, 1>::Zero();
    // 自碰撞

    Eigen::Matrix<double, 31, 1> C_collision_self, C_collision_outer, C_balance;
    C_collision_self.setZero();
    C_collision_outer.setZero();
    C_balance.setZero();
    // 已经在 update函数里面 更新了 球的坐标信息 ，这里直接计算的是
    dis_collision_self(dis_left_right, dis_left_T3, dis_left_T2, dis_right_T3, dis_right_T2);
    // 这里面负责计算 C_collisiuon_self
    {
        Eigen::Matrix<double, 17, 1> q, q_next, ddis_left_right, ddis_left_T2, ddis_left_T3, ddis_right_T2, ddis_right_T3;
        q << q_waist, q_left, q_right;
        double next_dis_left_right, next_dis_left_T2, next_dis_left_T3, next_dis_right_T2, next_dis_right_T3;
        for (int i = 0; i < 17; i++)
        {
            q_next = q;
            q_next[i] += 0.1;
            dis_collision_self(q_next, next_dis_left_right, next_dis_left_T3, next_dis_left_T2, next_dis_right_T3, next_dis_right_T2);

            // if (i == 0)
            //     cout << dis_left_right << ' ' << next_dis_left_right << endl;

            ddis_left_right[i] = (next_dis_left_right - dis_left_right) / 0.1;
            ddis_left_T3[i] = (next_dis_left_T3 - dis_left_T3) / 0.1;
            ddis_left_T2[i] = (next_dis_left_T2 - dis_left_T2) / 0.1;
            ddis_right_T3[i] = (next_dis_right_T3 - dis_right_T3) / 0.1;
            ddis_right_T2[i] = (next_dis_right_T2 - dis_right_T2) / 0.1;
        }

        Eigen::Matrix<double, 17, 1> c_left_right, c_left_T3, c_left_T2, c_right_T3, c_right_T2;
        // 根据距离 加权重  ,0.1米外不受影像 另外还有比例参数；
        double l1 = 10, l2 = 10, l3 = 10, l4 = 10, l5 = 10;
        if (dis_left_right > 0.1)
            c_left_right.setZero();
        else
            c_left_right = -l1 * ddis_left_right * (1 / (dis_left_right - 0.02) - 12.5);

        if (dis_left_T2 > 0.1)
            c_left_T2.setZero();
        else
            c_left_T2 = -l2 * ddis_left_T2 * (1 / (dis_left_T2 - 0.02) - 12.5);

        if (dis_left_T3 > 0.1)
            c_left_T3.setZero();
        else
            c_left_T3 = -l3 * ddis_left_T3 * (1 / (dis_left_T3 - 0.02) - 12.5);

        if (dis_right_T2 > 0.1)
            c_right_T2.setZero();
        else
            c_right_T2 = -l4 * ddis_right_T2 * (1 / (dis_right_T2 - 0.02) - 12.5);

        if (dis_right_T3 > 0.1)
            c_right_T3.setZero();
        else
            c_right_T3 = -l5 * ddis_right_T3 * (1 / (dis_right_T3 - 0.02) - 12.5);

        // 开始计算 C_collision_self
        C_collision_self.block(2, 0, 17, 1) = c_left_right + c_left_T3 + c_left_T2 + c_right_T3 + c_right_T2;
    }

    // 这里面 负责计算 C_collision_outer
    {
    }

    // 计算 C_balance
    {
        Eigen::Matrix<double, 17, 1> q, q0;
        q0 << q_waist, q_left, q_right;
        Eigen::Matrix<double, 17, 1> zero_q;
        zero_q.setZero();
        double x_initial = ZMP(q0, zero_q, zero_q)(0);

        cout << "x_initial" << x_initial << endl;
        if (-0.1 < x_initial && x_initial < 0.1)
            C_balance.setZero();
        else if (x_initial < -0.1)
        {
            for (int i = 0; i < 17; i++)
            {
                q = q0;
                q(i) += 0.1;
                double x = ZMP(q, zero_q, zero_q)(0);

                C_balance(2 + i) = -(x - x_initial) / 0.1 * (1 / (x_initial + 0.15) - 20);
            }
        }

        else if (x_initial > 0.1)
        {
            for (int i = 0; i < 17; i++)
            {
                q = q0;
                q(i) += 0.1;
                double x = ZMP(q, zero_q, zero_q)(0);

                C_balance(2 + i) = (x - x_initial) / 0.1 * (1 / (0.15 - x_initial) - 20);
            }
        }
    }

    // cout << "C_balance " << C_balance << endl;
    C = C + C_collision_outer + C_collision_self + C_balance;

    //  -------------------J---------------------------------

    Matrix<double, 3, 3> R_Carbase_base = T_Carbase_Base.block(0, 0, 3, 3);
    Matrix<double, 6, 6> big_R_Carbase_base = Matrix<double, 6, 6>::Zero();
    big_R_Carbase_base.block(0, 0, 3, 3) = R_Carbase_base;
    big_R_Carbase_base.block(3, 3, 3, 3) = R_Carbase_base;

    Matrix<double, 6, 10> J_Carbase_Leffector = big_R_Carbase_base * Jacobian(dh_list_Base_Leffector, q_Base_Leffector);
    Matrix<double, 6, 10> J_Carbase_Reffector = big_R_Carbase_base * Jacobian(dh_list_Base_Reffector, q_Base_Reffector);

    Matrix<double, 6, 2> a_part;
    a_part << 0, 1,
        0, 0,
        0, 0,
        0, 0,
        0, 0,
        1, 0;

    J.block(0, 0, 6, 2) = a_part;
    J.block(6, 0, 6, 2) = a_part;
    J.block(0, 2, 6, 10) = J_Carbase_Leffector;
    J.block(0, 12, 6, 7) = Matrix<double, 6, 7>::Zero();
    J.block(0, 19, 6, 6) = Matrix<double, 6, 6>::Identity();
    J.block(0, 25, 6, 6) = Matrix<double, 6, 6>::Zero();

    //
    J.block(6, 2, 6, 3) = J_Carbase_Reffector.block(0, 0, 6, 3);
    J.block(6, 5, 6, 7) = Matrix<double, 6, 7>::Zero();
    J.block(6, 12, 6, 7) = J_Carbase_Reffector.block(0, 3, 6, 7);
    J.block(6, 19, 6, 6) = Matrix<double, 6, 6>::Zero();
    J.block(6, 25, 6, 6) = Matrix<double, 6, 6>::Identity();

    // ---------------------------------------V_carbase_Leffector_star---------------------------------------------
    Eigen::Quaterniond quaternion_left(R_Leffector_Eleftstar); // 转四元数
    Eigen::Matrix<double, 3, 1> v_Leffector, w_Leffector;
    double theta = 2 * acos(quaternion_left.w());

    if (theta <= 0.001 || theta >= 2 * M_PI - 0.001)
    {
        w_Leffector = Matrix<double, 3, 1>::Zero();
    }
    else if (theta < M_PI)
    {
        w_Leffector << theta * quaternion_left.x() / sin(theta / 2), theta * quaternion_left.y() / sin(theta / 2), theta * quaternion_left.z() / sin(theta / 2);
    }
    else
    {
        w_Leffector << quaternion_left.x() / sin(theta / 2), quaternion_left.y() / sin(theta / 2), quaternion_left.z() / sin(theta / 2); // 原来的轴
        // 说明 轴 取反 ，角度 才能取 锐角
        theta = 2 * M_PI - theta;
        w_Leffector = -w_Leffector * theta;
    }

    v_Leffector = T_Leffector_Eleftstar.block(0, 3, 3, 1);

    dis_left_leftstar = sqrt(v_Leffector[0] * v_Leffector[0] + v_Leffector[1] * v_Leffector[1] + v_Leffector[2] * v_Leffector[2]);
    // has obtain  v_Leffector , w_Leffector

    // 一些 坐标变换
    Eigen::Matrix<double, 3, 3> R_Carbase_World = T_World_Carbase.block(0, 0, 3, 3).transpose();
    Eigen::Matrix<double, 6, 6> big_R_Carbase_World = Eigen::Matrix<double, 6, 6>::Zero();
    big_R_Carbase_World.block(0, 0, 3, 3) = R_Carbase_World;
    big_R_Carbase_World.block(3, 3, 3, 3) = R_Carbase_World;

    V_Carbase_Leffector_star << R_Carbase_Leffector * v_Leffector, R_Carbase_Leffector * w_Leffector;
    V_Carbase_Leffector_star = beta * V_Carbase_Leffector_star + big_R_Carbase_World * Velocity_World_leftstar; // big_R_Carbase_World * Velocity_World_leftstar 这个是 用于 处理 轨迹跟踪  额外的 速度 ；
    ////////
    Eigen::Quaterniond quaternion_right(R_Reffector_Erightstar); // 转四元数

    Eigen::Matrix<double, 3, 1> v_Reffector, w_Reffector;
    theta = 2 * acos(quaternion_right.w());

    if (theta <= 0.001 || theta >= 2 * M_PI - 0.001)
    {
        w_Reffector = Matrix<double, 3, 1>::Zero();
    }
    else if (theta < M_PI)
    {
        w_Reffector << theta * quaternion_right.x() / sin(theta / 2), theta * quaternion_right.y() / sin(theta / 2), theta * quaternion_right.z() / sin(theta / 2);
    }
    else
    {
        w_Reffector << quaternion_right.x() / sin(theta / 2), quaternion_right.y() / sin(theta / 2), quaternion_right.z() / sin(theta / 2); // 原来的轴
        // 说明 轴 取反 ，角度 才能取 锐角
        theta = 2 * M_PI - theta;
        w_Reffector = -w_Reffector * theta;
    }

    v_Reffector = T_Reffector_Erightstar.block(0, 3, 3, 1);

    dis_right_rightstar = sqrt(v_Reffector[0] * v_Reffector[0] + v_Reffector[1] * v_Reffector[1] + v_Reffector[2] * v_Reffector[2]);
    // has obtain  v_Leffector , w_Leffector

    V_Carbase_Reffector_star << R_Carbase_Reffector * v_Reffector, R_Carbase_Reffector * w_Reffector;

    V_Carbase_Reffector_star = beta * V_Carbase_Reffector_star + big_R_Carbase_World * Velocity_World_rightstar; // big_R_Carbase_World * Velocity_World_rightstar 用于轨迹跟踪的额外速度

    V_Carbase_all_star << V_Carbase_Leffector_star, V_Carbase_Reffector_star;

    //------------------------------A, B--------------------------------
    A = Matrix<double, 17, 31>::Zero();
    // 3 + 7 + 7  //  因为还有 waist
    B = 0.001 * Matrix<double, 17, 1>::Ones(); // 0.001 近似 0

    for (int i = 0; i < 3; i++)
    {
        if (q_waist(i) - q_waist_min(i) <= pi)
        {
            A(i, i + 2) = -1;
            B(i) = -eta * (q_waist_min(i) - q_waist(i) + ps) / (pi - ps);
        }
        else if (q_waist_max(i) - q_waist(i) <= pi)
        {
            A(i, i + 2) = 1;
            B(i) = eta * (q_waist_max(i) - q_waist(i) - ps) / (pi - ps);
        }
        // 饱和到 -gain 到 gain
        if (B(i) > eta)
            B(i) = eta;
        else if (B(i) < -eta)
            B(i) = -eta;
    }

    for (int i = 0; i < 7; i++)
    {
        if (q_left(i) - q_min(i) <= pi)
        {
            A(i + 3, i + 5) = -1;
            B(i + 3) = -eta * (q_min(i) - q_left(i) + ps) / (pi - ps);
        }
        else if (q_max(i) - q_left(i) <= pi)
        {
            A(i + 3, i + 5) = 1;
            B(i + 3) = eta * (q_max(i) - q_left(i) - ps) / (pi - ps);
        }
        // 饱和到 -gain 到 gain
        if (B(i + 3) > eta)
            B(i + 3) = eta;
        else if (B(i + 3) < -eta)
            B(i + 3) = -eta;
    }

    for (int i = 0; i < 7; i++)
    {
        if (q_right(i) - q_min(i) <= pi)
        {
            A(i + 10, i + 12) = -1;
            B(i + 10) = -eta * (q_min(i) - q_right(i) + ps) / (pi - ps);
        }
        else if (q_max(i) - q_right(i) <= pi)
        {
            A(i + 10, i + 12) = 1;
            B(i + 10) = eta * (q_max(i) - q_right(i) - ps) / (pi - ps);
        }
        // 饱和到 -gain 到 gain
        if (B(i + 10) > eta)
            B(i + 10) = eta;
        else if (B(i + 10) < -eta)
            B(i + 10) = -eta;
    }

    // ---------------X_low, X_up---------------------
    X_low << -0.5, -0.3, -0.3 * Eigen::Matrix<double, 3, 1>::Ones(), -0.3 * Eigen::Matrix<double, 7, 1>::Ones(),
        -0.3 * Eigen::Matrix<double, 7, 1>::Ones(), -1000 * Eigen::Matrix<double, 12, 1>::Ones();
    X_up = -X_low;

    // --------------------QP---------------------------

    Eigen::Matrix<double, 60, 1> low, up;
    low << V_Carbase_all_star, -1000000 * Eigen::Matrix<double, 17, 1>::Ones(), X_low;
    up << V_Carbase_all_star, B, X_up;

    Eigen::SparseMatrix<double> Q_sparse = Q.sparseView();
    Eigen::Matrix<double, 60, 31> constraint;
    constraint << J, A, Eigen::Matrix<double, 31, 31>::Identity();
    Eigen::SparseMatrix<double> constraint_sparse = constraint.sparseView();

    OsqpEigen::Solver solver;

    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(31);
    solver.data()->setNumberOfConstraints(60); // J_Carbase_Leffector A I

    solver.data()->setLinearConstraintsMatrix(constraint_sparse);
    solver.data()->setHessianMatrix(Q_sparse);
    solver.data()->setGradient(C);
    solver.data()->setLowerBound(low);
    solver.data()->setUpperBound(up);

    solver.settings()->setVerbosity(false); // 禁用日志输出
    if (!solver.initSolver())
    {
        std::cerr << "Solver initialization failed!" << std::endl;
    }

    // 求解问题
    if (!solver.solve())
    {
        std::cerr << "Failed to solve the problem!" << std::endl;
    }

    Eigen::VectorXd solution = solver.getSolution();

    // cout << "solution" << solution << endl;
    return solution.block(0, 0, 19, 1);
}

Eigen::Matrix<double, 19, 1> Car_arm::Solver2()
{
    // 解决问题 car + left arm
    Eigen::Matrix<double, 31, 1> x;
    Eigen::Matrix<double, 31, 31> Q;
    Eigen::Matrix<double, 31, 1> C;
    Eigen::Matrix<double, 12, 31> J;
    Eigen::Matrix<double, 6, 1> V_Carbase_Leffector_star, V_Carbase_Reffector_star; // desired velocity
    Eigen::Matrix<double, 12, 1> V_Carbase_all_star;
    Eigen::Matrix<double, 17, 31> A;
    Eigen::Matrix<double, 17, 1> B;
    Eigen::Matrix<double, 31, 1> X_low, X_up;
    // tackle axis transformation

    T_World_Leffector = T_World_Carbase * T_Carbase_Base * T_Base_Leffector;
    T_World_Reffector = T_World_Carbase * T_Carbase_Base * T_Base_Reffector;

    Eigen::Matrix<double, 4, 4> T_Leffector_Eleftstar = T_World_Leffector.inverse() * T_World_Eleftstar;
    Eigen::Matrix<double, 3, 3> R_Leffector_Eleftstar = T_Leffector_Eleftstar.block(0, 0, 3, 3);

    Eigen::Matrix<double, 4, 4> T_Carbase_Leffector = T_Carbase_Base * T_Base_Leffector;
    Eigen::Matrix<double, 3, 3> R_Carbase_Leffector = T_Carbase_Leffector.block(0, 0, 3, 3);

    //
    Eigen::Matrix<double, 4, 4> T_Reffector_Erightstar = T_World_Reffector.inverse() * T_World_Erightstar;
    Eigen::Matrix<double, 3, 3> R_Reffector_Erightstar = T_Reffector_Erightstar.block(0, 0, 3, 3);

    Eigen::Matrix<double, 4, 4> T_Carbase_Reffector = T_Carbase_Base * T_Base_Reffector;
    Eigen::Matrix<double, 3, 3> R_Carbase_Reffector = T_Carbase_Reffector.block(0, 0, 3, 3);

    // ---------------------Q-----------------------------------
    Eigen::VectorXd lambda_q(19), lambda_slack(12);

    lambda_q << 2 / (dis_Carbase_Eleftstar + dis_Carbase_Erightstar), 2 / (dis_Carbase_Eleftstar + dis_Carbase_Erightstar),
        5 * ka, 5 * ka, 5 * ka,     // waist
        ka, ka, ka, ka, ka, ka, ka, // left
        ka, ka, ka, ka, ka, ka, ka; // right

    lambda_slack << 1 / dis_Leffector_Eleftstar, 1 / dis_Leffector_Eleftstar, 1 / dis_Leffector_Eleftstar,
        0.1 / dis_Leffector_Eleftstar, 0.1 / dis_Leffector_Eleftstar, 0.1 / dis_Leffector_Eleftstar,

        1 / dis_Reffector_Erightstar, 1 / dis_Reffector_Erightstar, 1 / dis_Reffector_Erightstar,
        0.1 / dis_Reffector_Erightstar, 0.1 / dis_Reffector_Erightstar, 0.1 / dis_Reffector_Erightstar;

    Eigen::VectorXd diag_value(31);
    diag_value << lambda_q, lambda_slack;
    Q = Eigen::DiagonalMatrix<double, 31>(diag_value);

    // -------------------C---------------------------------
    Matrix<double, 10, 1> Jm_Base_Leffector = Manipulability(dh_list_Base_Leffector, q_Base_Leffector);
    Matrix<double, 10, 1> Jm_Base_Reffector = Manipulability(dh_list_Base_Reffector, q_Base_Reffector);
    C(0, 0) = 0;
    C.block(2, 0, 3, 1) = -Jm_Base_Leffector.block(0, 0, 3, 1) - Jm_Base_Reffector.block(0, 0, 3, 1);
    C.block(5, 0, 7, 1) = -Jm_Base_Leffector.block(3, 0, 7, 1);
    C.block(12, 0, 7, 1) = -Jm_Base_Reffector.block(3, 0, 7, 1);
    C.block(19, 0, 12, 1) = Eigen::Matrix<double, 12, 1>::Zero();
    // 自碰撞

    Eigen::Matrix<double, 31, 1> C_collision_self, C_collision_outer, C_balance;
    C_collision_self.setZero();
    C_collision_outer.setZero();
    C_balance.setZero();
    // 已经在 update函数里面 更新了 球的坐标信息 ，这里直接计算的是
    dis_collision_self(dis_left_right, dis_left_T3, dis_left_T2, dis_right_T3, dis_right_T2);
    // 这里面负责计算 C_collisiuon_self
    {
        Eigen::Matrix<double, 17, 1> q, q_next, ddis_left_right, ddis_left_T2, ddis_left_T3, ddis_right_T2, ddis_right_T3;
        q << q_waist, q_left, q_right;
        double next_dis_left_right, next_dis_left_T2, next_dis_left_T3, next_dis_right_T2, next_dis_right_T3;
        for (int i = 0; i < 17; i++)
        {
            q_next = q;
            q_next[i] += 0.1;
            dis_collision_self(q_next, next_dis_left_right, next_dis_left_T3, next_dis_left_T2, next_dis_right_T3, next_dis_right_T2);

            // if (i == 0)
            //     cout << dis_left_right << ' ' << next_dis_left_right << endl;

            ddis_left_right[i] = (next_dis_left_right - dis_left_right) / 0.1;
            ddis_left_T3[i] = (next_dis_left_T3 - dis_left_T3) / 0.1;
            ddis_left_T2[i] = (next_dis_left_T2 - dis_left_T2) / 0.1;
            ddis_right_T3[i] = (next_dis_right_T3 - dis_right_T3) / 0.1;
            ddis_right_T2[i] = (next_dis_right_T2 - dis_right_T2) / 0.1;
        }

        Eigen::Matrix<double, 17, 1> c_left_right, c_left_T3, c_left_T2, c_right_T3, c_right_T2;
        // 根据距离 加权重  ,0.1米外不受影像 另外还有比例参数；
        double l1 = 10, l2 = 10, l3 = 10, l4 = 10, l5 = 10;
        if (dis_left_right > 0.1)
            c_left_right.setZero();
        else
            c_left_right = -l1 * ddis_left_right * (1 / (dis_left_right - 0.02) - 12.5);

        if (dis_left_T2 > 0.1)
            c_left_T2.setZero();
        else
            c_left_T2 = -l2 * ddis_left_T2 * (1 / (dis_left_T2 - 0.02) - 12.5);

        if (dis_left_T3 > 0.1)
            c_left_T3.setZero();
        else
            c_left_T3 = -l3 * ddis_left_T3 * (1 / (dis_left_T3 - 0.02) - 12.5);

        if (dis_right_T2 > 0.1)
            c_right_T2.setZero();
        else
            c_right_T2 = -l4 * ddis_right_T2 * (1 / (dis_right_T2 - 0.02) - 12.5);

        if (dis_right_T3 > 0.1)
            c_right_T3.setZero();
        else
            c_right_T3 = -l5 * ddis_right_T3 * (1 / (dis_right_T3 - 0.02) - 12.5);

        // 开始计算 C_collision_self
        C_collision_self.block(2, 0, 17, 1) = c_left_right + c_left_T3 + c_left_T2 + c_right_T3 + c_right_T2;
    }

    // 这里面 负责计算 C_collision_outer
    {
    }

    // 计算 C_balance
    {
        Eigen::Matrix<double, 17, 1> q, q0;
        q0 << q_waist, q_left, q_right;
        Eigen::Matrix<double, 17, 1> zero_q;
        zero_q.setZero();
        double x_initial = ZMP(q0, zero_q, zero_q)(0);

        cout << "x_initial" << x_initial << endl;
        if (-0.1 < x_initial && x_initial < 0.1)
            C_balance.setZero();
        else if (x_initial < -0.1)
        {
            for (int i = 0; i < 17; i++)
            {
                q = q0;
                q(i) += 0.1;
                double x = ZMP(q, zero_q, zero_q)(0);

                C_balance(2 + i) = -(x - x_initial) / 0.1 * (1 / (x_initial + 0.15) - 20);
            }
        }

        else if (x_initial > 0.1)
        {
            for (int i = 0; i < 17; i++)
            {
                q = q0;
                q(i) += 0.1;
                double x = ZMP(q, zero_q, zero_q)(0);

                C_balance(2 + i) = (x - x_initial) / 0.1 * (1 / (0.15 - x_initial) - 20);
            }
        }
    }

    // cout << "C_balance " << C_balance << endl;
    C = C + C_collision_outer + C_collision_self + C_balance;

    //------------------------------A, B--------------------------------
    A = Matrix<double, 17, 31>::Zero();
    // 3 + 7 + 7  //  因为还有 waist
    B = 0.001 * Matrix<double, 17, 1>::Ones(); // 0.001 近似 0

    for (int i = 0; i < 3; i++)
    {
        if (q_waist(i) - q_waist_min(i) <= pi)
        {
            A(i, i + 2) = -1;
            B(i) = -eta * (q_waist_min(i) - q_waist(i) + ps) / (pi - ps);
        }
        else if (q_waist_max(i) - q_waist(i) <= pi)
        {
            A(i, i + 2) = 1;
            B(i) = eta * (q_waist_max(i) - q_waist(i) - ps) / (pi - ps);
        }
        // 饱和到 -gain 到 gain
        if (B(i) > eta)
            B(i) = eta;
        else if (B(i) < -eta)
            B(i) = -eta;
    }

    for (int i = 0; i < 7; i++)
    {
        if (q_left(i) - q_min(i) <= pi)
        {
            A(i + 3, i + 5) = -1;
            B(i + 3) = -eta * (q_min(i) - q_left(i) + ps) / (pi - ps);
        }
        else if (q_max(i) - q_left(i) <= pi)
        {
            A(i + 3, i + 5) = 1;
            B(i + 3) = eta * (q_max(i) - q_left(i) - ps) / (pi - ps);
        }
        // 饱和到 -gain 到 gain
        if (B(i + 3) > eta)
            B(i + 3) = eta;
        else if (B(i + 3) < -eta)
            B(i + 3) = -eta;
    }

    for (int i = 0; i < 7; i++)
    {
        if (q_right(i) - q_min(i) <= pi)
        {
            A(i + 10, i + 12) = -1;
            B(i + 10) = -eta * (q_min(i) - q_right(i) + ps) / (pi - ps);
        }
        else if (q_max(i) - q_right(i) <= pi)
        {
            A(i + 10, i + 12) = 1;
            B(i + 10) = eta * (q_max(i) - q_right(i) - ps) / (pi - ps);
        }
        // 饱和到 -gain 到 gain
        if (B(i + 10) > eta)
            B(i + 10) = eta;
        else if (B(i + 10) < -eta)
            B(i + 10) = -eta;
    }

    // ---------------X_low, X_up---------------------
    X_low << -0.5, -0.3, -0.3 * Eigen::Matrix<double, 3, 1>::Ones(), -0.3 * Eigen::Matrix<double, 7, 1>::Ones(),
        -0.3 * Eigen::Matrix<double, 7, 1>::Ones(), -1000 * Eigen::Matrix<double, 12, 1>::Ones();
    X_up = -X_low;

    // --------------------QP---------------------------

    Eigen::Matrix<double, 48, 1> low, up;
    low << -1000000 * Eigen::Matrix<double, 17, 1>::Ones(), X_low;
    up << B, X_up;

    Eigen::SparseMatrix<double> Q_sparse = Q.sparseView();
    Eigen::Matrix<double, 48, 31> constraint;
    constraint << A, Eigen::Matrix<double, 31, 31>::Identity();
    Eigen::SparseMatrix<double> constraint_sparse = constraint.sparseView();

    OsqpEigen::Solver solver;

    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(31);
    solver.data()->setNumberOfConstraints(48); // J_Carbase_Leffector A I

    solver.data()->setLinearConstraintsMatrix(constraint_sparse);
    solver.data()->setHessianMatrix(Q_sparse);
    solver.data()->setGradient(C);
    solver.data()->setLowerBound(low);
    solver.data()->setUpperBound(up);

    solver.settings()->setVerbosity(false); // 禁用日志输出
    if (!solver.initSolver())
    {
        std::cerr << "Solver initialization failed!" << std::endl;
    }

    // 求解问题
    if (!solver.solve())
    {
        std::cerr << "Failed to solve the problem!" << std::endl;
    }

    Eigen::VectorXd solution = solver.getSolution();

    // cout << "solution" << solution << endl;
    return solution.block(0, 0, 19, 1);
}

void Car_arm::Update_parameter(const Eigen::Matrix<double, 4, 4> &T_World_Carbase_input,
                               const Eigen::Matrix<double, 4, 4> &T_World_Eleftstar_input,
                               const Eigen::Matrix<double, 4, 4> &T_World_Erightstar_input,
                               const Eigen::Matrix<double, 6, 1> &Velocity_World_Eleftstar_input,
                               const Eigen::Matrix<double, 6, 1> &Velocity_World_Erightstar_input,
                               const Eigen::Matrix<double, 3, 1> &q_waist_input,
                               const Eigen::Matrix<double, 7, 1> &q_left_input,
                               const Eigen::Matrix<double, 7, 1> &q_right_input)
{

    q_waist = q_waist_input;
    q_left = q_left_input;
    q_right = q_right_input;
    // get  dis_Base  dis_Leffector theta_Carbase_Leffector theta_Carbase_Reffector
    T_World_Carbase = T_World_Carbase_input;
    T_World_Eleftstar = T_World_Eleftstar_input;
    T_World_Erightstar = T_World_Erightstar_input;

    Velocity_World_leftstar = Velocity_World_Eleftstar_input;
    Velocity_World_rightstar = Velocity_World_Erightstar_input;

    dis_Carbase_Eleftstar = sqrt((T_World_Carbase(0, 3) - T_World_Eleftstar(0, 3)) * (T_World_Carbase(0, 3) - T_World_Eleftstar(0, 3)) + (T_World_Carbase(1, 3) - T_World_Eleftstar(1, 3)) * (T_World_Carbase(1, 3) - T_World_Eleftstar(1, 3))); // 这里 只 计算了 平面距离
    dis_Carbase_Erightstar = sqrt((T_World_Carbase(0, 3) - T_World_Erightstar(0, 3)) * (T_World_Carbase(0, 3) - T_World_Erightstar(0, 3)) + (T_World_Carbase(1, 3) - T_World_Erightstar(1, 3)) * (T_World_Carbase(1, 3) - T_World_Erightstar(1, 3)));

    q_Base_Leffector << q_waist, q_left;
    q_Base_Reffector << q_waist, q_right;
    T_Base_Leffector = Fkine_collision(dh_list_Base_Leffector, q_Base_Leffector, 1) * T_leftend_tool;
    T_Base_Reffector = Fkine_collision(dh_list_Base_Reffector, q_Base_Reffector, 0) * T_rightend_tool;
    Eigen::Matrix<double, 4, 4> T_Carbase_Leffector = T_Carbase_Base * T_Base_Leffector;

    Eigen::Matrix<double, 4, 4> T_Carbase_Reffector = T_Carbase_Base * T_Base_Reffector;

    Eigen::Matrix<double, 4, 4> T_World_Leffector = T_World_Carbase * T_Carbase_Leffector;
    Eigen::Matrix<double, 4, 4> T_World_Reffector = T_World_Carbase * T_Carbase_Reffector;

    dis_Leffector_Eleftstar = sqrt((T_World_Leffector(0, 3) - T_World_Eleftstar(0, 3)) * (T_World_Leffector(0, 3) - T_World_Eleftstar(0, 3)) +
                                   (T_World_Leffector(1, 3) - T_World_Eleftstar(1, 3)) * (T_World_Leffector(1, 3) - T_World_Eleftstar(1, 3)) +
                                   (T_World_Leffector(2, 3) - T_World_Eleftstar(2, 3)) * (T_World_Leffector(2, 3) - T_World_Eleftstar(2, 3))); // 这里 只 计算了3d距离

    dis_Reffector_Erightstar = sqrt((T_World_Reffector(0, 3) - T_World_Erightstar(0, 3)) * (T_World_Reffector(0, 3) - T_World_Erightstar(0, 3)) +
                                    (T_World_Reffector(1, 3) - T_World_Erightstar(1, 3)) * (T_World_Reffector(1, 3) - T_World_Erightstar(1, 3)) +
                                    (T_World_Reffector(2, 3) - T_World_Erightstar(2, 3)) * (T_World_Reffector(2, 3) - T_World_Erightstar(2, 3))); // 这里 只 计算了3d距离

    if (dis_Carbase_Eleftstar < 0.5)
        dis_Carbase_Eleftstar = 0.5;
    else if (dis_Carbase_Eleftstar > 2)
        dis_Carbase_Eleftstar = 2;

    if (dis_Carbase_Erightstar < 0.5)
        dis_Carbase_Erightstar = 0.5;
    else if (dis_Carbase_Erightstar > 2)
        dis_Carbase_Erightstar = 2;

    if (dis_Leffector_Eleftstar < 0.01)
        dis_Leffector_Eleftstar = 0.01;

    if (dis_Reffector_Erightstar < 0.01)
        dis_Reffector_Erightstar = 0.01;

    auto get_theta = [=](const Eigen::Matrix<double, 4, 4> &T_Carbase_effector, double &theta, bool Isright)
    {
        Eigen::Matrix<double, 2, 1> a, b;
        if (!Isright)
        {
            b << 1, 0;
            a << T_Carbase_effector(0, 3), T_Carbase_effector(1, 3) - 0.3;
        }
        else
        {
            b << 1, -0;
            a << T_Carbase_effector(0, 3), T_Carbase_effector(1, 3) + 0.3;
        }

        double mu_a = sqrt(a(0) * a(0) + a(1) * a(1));
        if (mu_a < 0.01)
            theta = 0;
        else
        {
            theta = acos((a(0) * b(0) + a(1) * b(1)) / mu_a);
            if (a(1) < 0)
                theta = -theta;
        }
    };

    get_theta(T_Carbase_Leffector, theta_Carbase_Leffector, 0);
    get_theta(T_Carbase_Reffector, theta_Carbase_Reffector, 1);
}

//
