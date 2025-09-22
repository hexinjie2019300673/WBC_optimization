#include "car_arm.h"

Eigen::Matrix<double, 9, 1> Car_arm::Solver_single()
{
    // 解决问题 car + left arm
    Eigen::Matrix<double, 15, 1> x;
    Eigen::Matrix<double, 15, 15> Q;
    Eigen::Matrix<double, 15, 1> C;
    Eigen::Matrix<double, 6, 15> J;
    Eigen::Matrix<double, 6, 1> V_Carbase_Leffector_star;
    Eigen::Matrix<double, 7, 15> A;
    Eigen::Matrix<double, 7, 1> B;
    Eigen::Matrix<double, 15, 1> X_low, X_up;
    // tackle axis transformation

    Eigen::Matrix<double, 10, 1> q_all;
    q_all << q_waist, q_left;
    Eigen::Matrix<double, 4, 4> T_Base_Leffector = Fkine(dh_list_Base_Leffector, q_all);

    T_World_Leffector = T_World_Carbase * T_Carbase_Base * T_Base_Leffector;
    Eigen::Matrix<double, 4, 4> T_Leffector_Estar = T_World_Leffector.inverse() * T_World_Eleftstar;

    Eigen::Matrix<double, 3, 3> R_Leffector_Estar = T_Leffector_Estar.block(0, 0, 3, 3);

    Eigen::Matrix<double, 4, 4> T_Carbase_Leffector = T_Carbase_Base * T_Base_Leffector;
    Eigen::Matrix<double, 3, 3> R_Carbase_Leffector = T_Carbase_Leffector.block(0, 0, 3, 3);

    // ---------------------Q-----------------------------------
    Eigen::VectorXd lambda_q(9), lambda_slack(6);

    lambda_q << 6 / dis_Carbase_Eleftstar, 2 / dis_Carbase_Eleftstar, ka, ka, ka, ka, ka, ka, ka; // 车 2 维 ， 臂 7 维
    lambda_slack << 1 / dis_Leffector_Eleftstar, 1 / dis_Leffector_Eleftstar, 1 / dis_Leffector_Eleftstar, 0.1 / dis_Leffector_Eleftstar, 0.1 / dis_Leffector_Eleftstar, 0.1 / dis_Leffector_Eleftstar;
    Eigen::VectorXd diag_value(15);
    diag_value << lambda_q, lambda_slack;
    Q = Eigen::DiagonalMatrix<double, 15>(diag_value);

    // -------------------C---------------------------------
    // manipulability matrix
    Matrix<double, 7, 1> Jm_Larm = Manipulability(dh_list_Lbase_Leffector, q_left);
    // Matrix<double, 10, 1> q_temporary;
    // q_temporary << 0, 0, 0, q_left;
    // Matrix<double, 7, 1> Jm_Larm = Manipulability(dh_list_Base_Leffector, q_temporary).block(3, 0, 7, 1);
    // 暂时 左臂

    Eigen::VectorXd Jm(9), Episilon(9);
    Jm << 0, 0, Jm_Larm;

    Episilon = Matrix<double, 9, 1>::Zero();
    Episilon(0) = -k_episilon * theta_Carbase_Leffector;
    cout << "theta" << theta_Carbase_Leffector << endl;
    C.block(0, 0, 9, 1) = -Jm + Episilon;
    // C.block(0, 0, 9, 1) = -Jm;
    cout << "theta_Carbase_Leffector" << theta_Carbase_Leffector << endl;
    //  -------------------J---------------------------------
    Matrix<double, 6, 7> J_Lbase_Leffector = Jacobian(dh_list_Lbase_Leffector, q_left);

    Matrix<double, 3, 1> q_waist = Matrix<double, 3, 1>::Zero();
    Matrix<double, 3, 3> R_base_Lbase = Fkine(dh_list_Waist, q_waist).block(0, 0, 3, 3);

    Matrix<double, 3, 3> R_Carbase_base = T_Carbase_Base.block(0, 0, 3, 3);
    Matrix<double, 3, 3> R_Carbase_Lbase = R_Carbase_base * R_base_Lbase;
    Matrix<double, 6, 6> Transform_Carbase_Lbase = Matrix<double, 6, 6>::Identity();
    Transform_Carbase_Lbase.block(0, 0, 3, 3) = R_Carbase_Lbase;
    Transform_Carbase_Lbase.block(3, 3, 3, 3) = R_Carbase_Lbase;
    Matrix<double, 6, 7> J_Carbase_Leffector = Transform_Carbase_Lbase * J_Lbase_Leffector;

    Matrix<double, 6, 9> J_Carbase_Leffector_includeCar;
    J_Carbase_Leffector_includeCar.block(0, 2, 6, 7) = J_Carbase_Leffector;

    J_Carbase_Leffector_includeCar.block(0, 0, 6, 2) << 0, 1,
        0, 0,
        0, 0,
        0, 0,
        0, 0,
        1, 0;

    J << J_Carbase_Leffector_includeCar, Matrix<double, 6, 6>::Identity();

    // -------------------------V_carbase_Leffector_star---------------------------------------------
    Eigen::Quaterniond quaternion(R_Leffector_Estar); // 转四元数

    Eigen::Matrix<double, 3, 1> v_Leffector, w_Leffector;

    double theta = 2 * acos(quaternion.w());

    if (theta <= 0.001 || theta >= 2 * M_PI - 0.001)
    {
        w_Leffector = Matrix<double, 3, 1>::Zero();
    }
    else if (theta < M_PI)
    {
        w_Leffector << theta * quaternion.x() / sin(theta / 2), theta * quaternion.y() / sin(theta / 2), theta * quaternion.z() / sin(theta / 2);
    }
    else
    {
        w_Leffector << quaternion.x() / sin(theta / 2), quaternion.y() / sin(theta / 2), quaternion.z() / sin(theta / 2); // 原来的轴
        // 说明 轴 取反 ，角度 才能取 锐角
        theta = 2 * M_PI - theta;
        w_Leffector = -w_Leffector * theta;
    }

    v_Leffector = T_Leffector_Estar.block(0, 3, 3, 1);
    // has obtain  v_Leffector , w_Leffector

    V_Carbase_Leffector_star << R_Carbase_Leffector * v_Leffector, R_Carbase_Leffector * w_Leffector;
    V_Carbase_Leffector_star = beta * V_Carbase_Leffector_star;

    //---------------------A, B--------------------------------
    A = Matrix<double, 7, 15>::Zero();
    B = 0.001 * Matrix<double, 7, 1>::Ones();

    for (int i = 0; i < 7; i++)
    {
        if (q_left(i) - q_min(i) <= pi)
        {
            A(i, i + 2) = -1;
            B(i) = -eta * (q_min(i) - q_left(i) + ps) / (pi - ps);
        }
        else if (q_max(i) - q_left(i) <= pi)
        {
            A(i, i + 2) = 1;
            B(i) = eta * (q_max(i) - q_left(i) - ps) / (pi - ps);
        }
        // 饱和到 -gain 到 gain
        if (B(i) > eta)
            B(i) = eta;
        else if (B(i) < -eta)
            B(i) = -eta;
    }

    // ---------------X_low, X_up---------------------
    X_low << -0.5, -0.3, -0.3 * Eigen::Matrix<double, 7, 1>::Ones(), -1000 * Eigen::Matrix<double, 6, 1>::Ones(); // car + arm + slack
    X_up = -X_low;

    // --------------------QP---------------------------

    Eigen::Matrix<double, 6 + 7 + 15, 1> low, up;
    low << V_Carbase_Leffector_star, -1000000 * Eigen::Matrix<double, 7, 1>::Ones(), X_low;
    up << V_Carbase_Leffector_star, B, X_up;

    Eigen::SparseMatrix<double> Q_sparse = Q.sparseView();
    Eigen::Matrix<double, 6 + 7 + 15, 15> constraint;
    constraint << J, A, Eigen::Matrix<double, 15, 15>::Identity();
    Eigen::SparseMatrix<double> constraint_sparse = constraint.sparseView();

    OsqpEigen::Solver solver;

    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(15);
    solver.data()->setNumberOfConstraints(6 + 7 + 15); // J_Carbase_Leffector A I

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

    return solution.block(0, 0, 9, 1);
}

void Car_arm::Update_parameter_single(const Eigen::Matrix<double, 4, 4> &T_World_Carbase_input,
                                      const Eigen::Matrix<double, 4, 4> &T_World_Estar_input,
                                      const Eigen::Matrix<double, 3, 1> &q_waist_input,
                                      const Eigen::Matrix<double, 7, 1> &q_left_input,
                                      const Eigen::Matrix<double, 7, 1> &q_right_input)
{

    q_waist = q_waist_input;
    q_left = q_left_input;
    q_right = q_right_input;
    // get  dis_Base  dis_Leffector theta_Carbase_Leffector theta_Carbase_Reffector
    T_World_Carbase = T_World_Carbase_input;
    T_World_Eleftstar = T_World_Estar_input;
    dis_Carbase_Eleftstar = sqrt((T_World_Carbase(0, 3) - T_World_Eleftstar(0, 3)) * (T_World_Carbase(0, 3) - T_World_Eleftstar(0, 3)) + (T_World_Carbase(1, 3) - T_World_Eleftstar(1, 3)) * (T_World_Carbase(1, 3) - T_World_Eleftstar(1, 3))); // 这里 只 计算了 平面距离

    Eigen::Matrix<double, 10, 1> q_base_Leffector, q_base_Reffector;
    q_base_Leffector << q_waist, q_left;
    q_base_Reffector << q_waist, q_right;
    Eigen::Matrix<double, 4, 4> T_Carbase_Leffector = T_Carbase_Base * Fkine(dh_list_Base_Leffector, q_base_Leffector);
    // cout << T_Carbase_Base << endl;
    // cout << Fkine(dh_list_Base_Leffector, q_base_Leffector) << endl;
    // cout << "T_Carbase_Leffector" << T_Carbase_Leffector(0, 3) << "," << T_Carbase_Leffector(1, 3) << "," << T_Carbase_Leffector(2, 3) << endl;

    Eigen::Matrix<double, 4, 4> T_Carbase_Reffector = T_Carbase_Base * Fkine(dh_list_Base_Reffector, q_base_Reffector);

    Eigen::Matrix<double, 4, 4> T_World_Leffector = T_World_Carbase * T_Carbase_Leffector;

    // cout << "T_World_Leffector" << T_World_Leffector(0, 3) << "," << T_World_Leffector(1, 3) << "," << T_World_Leffector(2, 3) << endl;
    // cout << "T_World_Estar" << T_World_Estar(0, 3) << "," << T_World_Estar(1, 3) << "," << T_World_Estar(2, 3) << endl;

    dis_Leffector_Eleftstar = sqrt((T_World_Leffector(0, 3) - T_World_Eleftstar(0, 3)) * (T_World_Leffector(0, 3) - T_World_Eleftstar(0, 3)) +
                                   (T_World_Leffector(1, 3) - T_World_Eleftstar(1, 3)) * (T_World_Leffector(1, 3) - T_World_Eleftstar(1, 3)) + (T_World_Leffector(2, 3) - T_World_Eleftstar(2, 3)) * (T_World_Leffector(2, 3) - T_World_Eleftstar(2, 3))); // 这里 只 计算了 平面距离

    if (dis_Carbase_Eleftstar < 0.5)
        dis_Carbase_Eleftstar = 0.5;
    else if (dis_Carbase_Eleftstar > 2)
        dis_Carbase_Eleftstar = 2;
    if (dis_Leffector_Eleftstar < 0.1)
        dis_Leffector_Eleftstar = 0.1;

    auto get_theta = [=](const Eigen::Matrix<double, 4, 4> &T_Carbase_effector, double &theta)
    {
        Eigen::Matrix<double, 2, 1> a, b;
        b << 0.86, 0.5;
        a << T_Carbase_effector(0, 3), T_Carbase_effector(1, 3) - 0.3;

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

    get_theta(T_Carbase_Leffector, theta_Carbase_Leffector);
    get_theta(T_Carbase_Reffector, theta_Carbase_Reffector);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
