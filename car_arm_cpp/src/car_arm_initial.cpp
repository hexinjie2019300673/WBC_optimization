#include "car_arm.h"

Car_arm::Car_arm()
{
    beta = 1, pi = 0.1, ps = 0.9, ka = 0.01, k_episilon = 0.5, eta = 1;
    q_min << -3.05, -2.09, -3.05, -2.09, -3.05, -2.09, -3.05;
    q_max << 3.05, 2.09, 3.05, 2.09, 3.05, 2.09, 3.05;
    q_waist_min << -M_PI / 2, -M_PI / 2, -M_PI / 2;
    q_waist_max << M_PI / 2, M_PI / 2, M_PI / 2;

    T_Carbase_Base << 0, 1, 0, -0.1295,
        0, 0, 1, 0,
        1, 0, 0, 0.227,
        0, 0, 0, 1;
    q_left.setZero();
    q_right.setZero();
    q_waist.setZero();
    q_car.setZero();
    // alpha, a, d, theta (MDH Parameters)

    // 末端执行器 相对于 link7 的位置
    // T_leftend_tool
    //     << 0,
    //     0, 1, 0.087,
    //     0, 1, 0, 0,
    //     -1, 0, 0, 0,
    //     0, 0, 0, 1;
    // T_rightend_tool << 0, 0, 1, 0.087,
    //     0, 1, 0, 0,
    //     -1, 0, 0, 0,
    //     0, 0, 0, 1;

    T_leftend_tool << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    T_rightend_tool << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    dh_list_Lbase_Leffector = {
        {-M_PI / 2, 0, 0.2197, 0}, // 左臂
        {M_PI / 2, 0, 0, 0},
        {-M_PI / 2, 0, 0.3005, 0},
        {M_PI / 2, 0.0225, 0, 0},
        {-M_PI / 2, -0.0225, 0.25, 0},
        {M_PI / 2, 0, 0, M_PI / 2},
        {-M_PI / 2, 0, 0, 0},
    };

    dh_list_Rbase_Reffector = {
        {M_PI / 2, 0, 0.2197, 0}, // 右臂
        {-M_PI / 2, 0, 0, 0},
        {M_PI / 2, 0, 0.3005, 0},
        {-M_PI / 2, 0.0225, 0, 0},
        {M_PI / 2, -0.0225, 0.25, 0},
        {-M_PI / 2, 0, 0, -M_PI / 2},
        {-M_PI / 2, 0, 0, 0},
    };

    dh_list_Waist =
        {
            {0, 0, 0, 0}, // 躯干
            {0, 0.32, 0, M_PI / 2},
            {M_PI / 2, 0, 0.5909, 0},
        };

    dh_list_Base_Leffector = {
        {0, 0, 0, 0}, // 躯干
        {0, 0.32, 0, M_PI / 2},
        {M_PI / 2, 0, 0.5909, 0},
        {-M_PI / 2, 0, 0.2297, 0}, // 左臂
        {M_PI / 2, 0, 0, 0},
        {-M_PI / 2, 0, 0.3005, 0},
        {M_PI / 2, 0.0225, 0, 0},
        {-M_PI / 2, -0.0225, 0.25, 0},
        {M_PI / 2, 0, 0, M_PI / 2},
        {-M_PI / 2, 0, 0, 0},
    };

    dh_list_Base_Reffector = {
        {0, 0, 0, 0}, // 躯干
        {0, 0.32, 0, M_PI / 2},
        {M_PI / 2, 0, 0.5909, 0},
        {M_PI / 2, 0, 0.2297, 0}, // 右臂
        {-M_PI / 2, 0, 0, 0},
        {M_PI / 2, 0, 0.3005, 0},
        {-M_PI / 2, 0.0225, 0, 0},
        {M_PI / 2, -0.0225, 0.25, 0},
        {-M_PI / 2, 0, 0, -M_PI / 2},
        {-M_PI / 2, 0, 0, 0},
    };

    // 下面是 动力学参数部分
    // ------------------------------------质量-----------------------------------
    Waist_Mass = {6.27, 3.82, 9.27};
    Left_Mass = {1.723, 1.309, 1.301, 0.626, 0.529, 0.502, 0.17};
    Right_Mass = {1.723, 1.309, 1.301, 0.626, 0.529, 0.502, 0.17};

    //------------------------------------- 质心----------------------------------
    Waist_MassCenter = {
        0.001 * Eigen::Vector3d(218.73, -0.04, -2.73),
        0.001 * Eigen::Vector3d(-0.01, -131.85, 3.52),
        0.001 * Eigen::Vector3d(-5.32, 7.13, -113.93)};

    Left_MassCenter = {
        0.001 * Eigen::Vector3d(0.01, -3.81, -5.31),
        0.001 * Eigen::Vector3d(-0.08, 95.31, -2.29),
        0.001 * Eigen::Vector3d(14.97, -7.81, -29.6),
        0.001 * Eigen::Vector3d(-19.01, 65.8, 16.47),
        0.001 * Eigen::Vector3d(0.52, -3.29, -81.36),
        0.001 * Eigen::Vector3d(0.02, 3.39, 0.47),
        0.001 * Eigen::Vector3d(54.49, -0.08, 5.6)};

    Right_MassCenter = {
        0.001 * Eigen::Vector3d(-0.01, 3.81, -5.31),
        0.001 * Eigen::Vector3d(0.08, -95.31, -2.29),
        0.001 * Eigen::Vector3d(-14.99, -7.88, -29.6),
        0.001 * Eigen::Vector3d(-19.01, -65.8, 16.47),
        0.001 * Eigen::Vector3d(-0.52, 3.29, -81.36),
        0.001 * Eigen::Vector3d(0.02, 3.39, 0.47),
        0.001 * Eigen::Vector3d(54.49, -0.08, 5.6)};

    //---------------------------------- 转动惯量 ------------------------------------
    Waist_Inertia = {
        (Eigen::Matrix3d() << 104866455.5, -29555.19, -638.63,
         -29555.19, 11871826.78, -7390319.41,
         -638.63, -7390319.41, 107344145.34)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 28291998.13, 1838.37, 1987.32,
         1838.37, 10839491.76, -1725391.65,
         1987.32, -1725391.65, -23813771.91)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 232875575.26, 2264907.46, 314967.23,
         2264907.46, 160773813.17, 3380308.43,
         314967.23, 3380308.43, 92962419.75)
                .finished() *
            1e-9};

    Left_Inertia = {
        (Eigen::Matrix3d() << 2266280.13, -4367.13, -8699.93,
         -4367.13, 2164645.79, 57744.46,
         -8699.93, 57744.46, 2043349.87)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 4359958.84, -3327.38, 1023.87,
         -3327.38, 2202730.03, -193907,
         1023.87, -193907.84, 3708100.09)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 4299183.89, 74620.16, -596022.26,
         74620.16, 4356901.35, 298994.07,
         -596022.36, 298994.07, 1370953.81)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 1113905.16, 116162.81, -76560.39,
         116162.81, 743211.48, 320390.71,
         -76560.39, 320390.71, 1188367.98)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 718032, -9553.7, -9252.67,
         -9553.7, 620500.81, -11415.91,
         -9252.67, -11415.91, 425731.53)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 445112.48, 1205.34, 95.71,
         1205.34, 302253.44, 2595.9,
         95.71, 2595.9, 430995.93)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 128774.8, 197.04, 34997.01,
         197.04, 147492.48, 38.02,
         34997.01, 38.02, 131922.97)
                .finished() *
            1e-9};

    Right_Inertia = {
        (Eigen::Matrix3d() << 2266280.13, -4367.13, 8699.93,
         -4367.13, 2164645.79, -57744.46,
         8699.93, -57744.46, 2043349.87)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 4359958.84, -3327.38, -1023.87,
         -3327.38, 2202730.03, 193907,
         -1023.87, 193907.84, 3708100.09)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 4299183.89, -78934.37, 596450.27,
         -78934.37, 4356440.97, 304574.97,
         596450.27, 304574.97, 1371748.18)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 1113905.16, -114124.01, -76942.60,
         -114124.01, 743115.54, -320363.93,
         -76942.60, -320363.54, 1188280.77)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 718032, 9553.7, -9252.67,
         9553.7, 620500.81, 11415.91,
         -9252.67, 11415.91, 425731.53)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 445112.48, 1205.34, -95.71,
         1205.34, 302253.44, -2595.9,
         -95.71, -2595.9, 430995.93)
                .finished() *
            1e-9,

        (Eigen::Matrix3d() << 128774.8, -197.04, 34997.01,
         -197.04, 147492.48, -38.02,
         34997.01, -38.02, 131922.97)
                .finished() *
            1e-9};
}

Eigen::Matrix<double, 4, 4> Car_arm::Fkine(const std::vector<std::vector<double>> &dh_list, const Eigen::Matrix<double, 10, 1> &q)
{
    if (size(dh_list) != 10)
    {
        std::cerr << "dimension error" << std::endl;
    }
    Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

    for (size_t i = 0; i < dh_list.size(); ++i)
    {
        double alpha = dh_list[i][0];
        double a = dh_list[i][1];
        double d = dh_list[i][2];
        double theta = dh_list[i][3] + q(i);

        Eigen::Matrix<double, 4, 4> A;
        A << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;

        T = T * A;
    }

    return T;
}

// 这个 Fkine_collision 在 计算 末端执行器的位恣时 顺带要计算 碰撞球的相对机器人base（不是底盘基座）的
Eigen::Matrix<double, 4, 4> Car_arm::Fkine(const std::vector<std::vector<double>> &dh_list, const Eigen::Matrix<double, 7, 1> &q)
{
    if (size(dh_list) != 7)
    {
        std::cerr << "dimension error" << std::endl;
    }

    Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

    for (size_t i = 0; i < dh_list.size(); ++i)
    {
        double alpha = dh_list[i][0];
        double a = dh_list[i][1];
        double d = dh_list[i][2];
        double theta = dh_list[i][3] + q(i);

        Eigen::Matrix<double, 4, 4> A;
        A << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;

        T = T * A;
    }

    return T;
}

Eigen::Matrix<double, 4, 4> Car_arm::Fkine(const std::vector<std::vector<double>> &dh_list, const Eigen::Matrix<double, 3, 1> &q)
{
    if (size(dh_list) != 3)
    {
        std::cerr << "dimension error" << std::endl;
    }

    Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

    for (size_t i = 0; i < dh_list.size(); ++i)
    {
        double alpha = dh_list[i][0];
        double a = dh_list[i][1];
        double d = dh_list[i][2];
        double theta = dh_list[i][3] + q(i);

        Eigen::Matrix<double, 4, 4> A;
        A << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;

        T = T * A;
    }

    return T;
}

Eigen::Matrix<double, 6, 10> Car_arm::Jacobian(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 10, 1> &q)
{
    if (size(dh_list) != 10)
    {
        std::cerr << "dimension error" << std::endl;
    }
    const int n = q.size();
    MatrixXd J(6, n);
    J.setZero();

    vector<Matrix4d> T_all(dh_list.size() + 1);
    T_all[0] = Matrix4d::Identity();

    // 1. 计算所有连杆的变换矩阵
    for (int i = 0; i < dh_list.size(); ++i)
    {
        double alpha = dh_list[i][0];
        double a = dh_list[i][1];
        double d = dh_list[i][2];
        double theta = dh_list[i][3] + q(i);

        Matrix4d A;
        A << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;
        T_all[i + 1] = T_all[i] * A;
    }

    const Vector3d p_ee = T_all.back().block<3, 1>(0, 3); // 末端位置

    // 2. 计算雅可比各列
    for (int i = 0; i < n; ++i)
    {
        const Vector3d z_i = T_all[i + 1].block<3, 1>(0, 2); // 关节i的Z轴方向
        const Vector3d p_i = T_all[i + 1].block<3, 1>(0, 3); // 关节i的位置

        // 假设所有关节均为旋转关节（可根据实际需求修改）
        J.block<3, 1>(3, i) = z_i;                   // 角速度部分
        J.block<3, 1>(0, i) = z_i.cross(p_ee - p_i); // 线速度部分
    }

    return J;
}

Eigen::Matrix<double, 6, 7> Car_arm::Jacobian(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 7, 1> &q)
{
    if (size(dh_list) != 7)
    {
        std::cerr << "dimension error" << std::endl;
    }
    const int n = q.size();
    MatrixXd J(6, n);
    J.setZero();

    vector<Matrix4d> T_all(dh_list.size() + 1);
    T_all[0] = Matrix4d::Identity();

    // 1. 计算所有连杆的变换矩阵
    for (int i = 0; i < dh_list.size(); ++i)
    {
        double alpha = dh_list[i][0];
        double a = dh_list[i][1];
        double d = dh_list[i][2];
        double theta = dh_list[i][3] + q(i);

        Matrix4d A;
        A << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;
        T_all[i + 1] = T_all[i] * A;
    }

    const Vector3d p_ee = T_all.back().block<3, 1>(0, 3); // 末端位置

    // 2. 计算雅可比各列
    for (int i = 0; i < n; ++i)
    {
        const Vector3d z_i = T_all[i + 1].block<3, 1>(0, 2); // 关节i的Z轴方向
        const Vector3d p_i = T_all[i + 1].block<3, 1>(0, 3); // 关节i的位置

        // 假设所有关节均为旋转关节（可根据实际需求修改）
        J.block<3, 1>(3, i) = z_i;                   // 角速度部分
        J.block<3, 1>(0, i) = z_i.cross(p_ee - p_i); // 线速度部分
    }

    return J;
}

Eigen::Matrix<double, 6, 3> Car_arm::Jacobian(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 3, 1> &q)
{
    if (size(dh_list) != 3)
    {
        std::cerr << "dimension error" << std::endl;
    }
    const int n = q.size();
    MatrixXd J(6, n);
    J.setZero();

    vector<Matrix4d> T_all(dh_list.size() + 1);
    T_all[0] = Matrix4d::Identity();

    // 1. 计算所有连杆的变换矩阵
    for (int i = 0; i < dh_list.size(); ++i)
    {
        double alpha = dh_list[i][0];
        double a = dh_list[i][1];
        double d = dh_list[i][2];
        double theta = dh_list[i][3] + q(i);

        Matrix4d A;
        A << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;
        T_all[i + 1] = T_all[i] * A;
    }

    const Vector3d p_ee = T_all.back().block<3, 1>(0, 3); // 末端位置

    // 2. 计算雅可比各列
    for (int i = 0; i < n; ++i)
    {
        const Vector3d z_i = T_all[i + 1].block<3, 1>(0, 2); // 关节i的Z轴方向
        const Vector3d p_i = T_all[i + 1].block<3, 1>(0, 3); // 关节i的位置

        // 假设所有关节均为旋转关节（可根据实际需求修改）
        J.block<3, 1>(3, i) = z_i;                   // 角速度部分
        J.block<3, 1>(0, i) = z_i.cross(p_ee - p_i); // 线速度部分
    }

    return J;
};

Eigen::Matrix<double, 3, 1> Car_arm::Manipulability(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 3, 1> &q)
{

    if (size(dh_list) != 3)
    {
        std::cerr << "dimension error" << std::endl;
    }
    Eigen::Matrix<double, 6, 3> jacobian_eigen = Jacobian(dh_list, q);

    Eigen::Matrix<double, 3, 3> J_trans = jacobian_eigen.block(0, 0, 3, 3);
    double manipulability = sqrt((J_trans * J_trans.transpose()).determinant());

    Eigen::Matrix<double, 3, 3> B = (J_trans * J_trans.transpose()).inverse();

    std::vector<Eigen::Matrix<double, 3, 3>> H(3, Eigen::Matrix<double, 3, 3>::Zero());

    for (int i = 0; i < 3; i++)
        for (int j = i; j < 3; j++)
        {
            Vector3d a;
            Vector3d b;

            a << jacobian_eigen(3, i), jacobian_eigen(4, i), jacobian_eigen(5, i);
            b << jacobian_eigen(0, j), jacobian_eigen(1, j), jacobian_eigen(2, j);
            Matrix<double, 3, 1> h;
            h << a.y() * b.z() - a.z() * b.y(),
                a.z() * b.x() - a.x() * b.z(),
                a.x() * b.y() - a.y() * b.x();

            for (int k = 0; k < 3; k++)
            {
                H[k](j, i) = h(k);
                H[k](i, j) = h(k);
            }
        }

    Matrix<double, 3, 1> Jm;

    for (int i = 0; i < 3; i++)
    {
        Matrix<double, 3, 3> H0;
        for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++)
                H0(a, b) = H[a](b, i);

        Eigen::Matrix<double, 3, 3> C = J_trans * H0.transpose();

        Eigen::Map<Eigen::VectorXd> C_reshape(C.data(), C.size());
        Eigen::Map<Eigen::VectorXd> B_reshape(B.data(), B.size());
        Jm(i) = manipulability * C_reshape.transpose() * B_reshape;
    }
    return Jm;
}

Eigen::Matrix<double, 7, 1> Car_arm::Manipulability(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 7, 1> &q)
{

    if (size(dh_list) != 7)
    {
        std::cerr << "dimension error" << std::endl;
    }
    Eigen::Matrix<double, 6, 7> jacobian_eigen = Jacobian(dh_list, q);

    Eigen::Matrix<double, 3, 7> J_trans = jacobian_eigen.block(0, 0, 3, 7);
    double manipulability = sqrt((J_trans * J_trans.transpose()).determinant());

    Eigen::Matrix<double, 3, 3> B = (J_trans * J_trans.transpose()).inverse();

    std::vector<Eigen::Matrix<double, 7, 7>> H(3, Eigen::Matrix<double, 7, 7>::Zero());

    for (int i = 0; i < 7; i++)
        for (int j = i; j < 7; j++)
        {
            Vector3d a;
            Vector3d b;

            a << jacobian_eigen(3, i), jacobian_eigen(4, i), jacobian_eigen(5, i);
            b << jacobian_eigen(0, j), jacobian_eigen(1, j), jacobian_eigen(2, j);
            Matrix<double, 3, 1> h;
            h << a.y() * b.z() - a.z() * b.y(),
                a.z() * b.x() - a.x() * b.z(),
                a.x() * b.y() - a.y() * b.x();

            for (int k = 0; k < 3; k++)
            {
                H[k](j, i) = h(k);
                H[k](i, j) = h(k);
            }
        }

    Matrix<double, 7, 1> Jm;

    for (int i = 0; i < 7; i++)
    {
        Matrix<double, 3, 7> H0;
        for (int a = 0; a < 3; a++)
            for (int b = 0; b < 7; b++)
                H0(a, b) = H[a](b, i);

        Eigen::Matrix<double, 3, 3> C = J_trans * H0.transpose();

        Eigen::Map<Eigen::VectorXd> C_reshape(C.data(), C.size());
        Eigen::Map<Eigen::VectorXd> B_reshape(B.data(), B.size());
        Jm(i) = manipulability * C_reshape.transpose() * B_reshape;
    }
    return Jm;
}

Eigen::Matrix<double, 10, 1> Car_arm::Manipulability(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 10, 1> &q)
{

    if (size(dh_list) != 10)
    {
        std::cerr << "dimension error" << std::endl;
    }
    Eigen::Matrix<double, 6, 10> jacobian_eigen = Jacobian(dh_list, q);

    Eigen::Matrix<double, 3, 10> J_trans = jacobian_eigen.block(0, 0, 3, 10);

    double manipulability = sqrt((J_trans * J_trans.transpose()).determinant());

    Eigen::Matrix<double, 3, 3> B = (J_trans * J_trans.transpose()).inverse();

    std::vector<Eigen::Matrix<double, 10, 10>> H(3, Eigen::Matrix<double, 10, 10>::Zero());

    for (int i = 0; i < 10; i++)
        for (int j = i; j < 10; j++)
        {
            Vector3d a;
            Vector3d b;

            a << jacobian_eigen(3, i), jacobian_eigen(4, i), jacobian_eigen(5, i);
            b << jacobian_eigen(0, j), jacobian_eigen(1, j), jacobian_eigen(2, j);
            Matrix<double, 3, 1> h;
            h << a.y() * b.z() - a.z() * b.y(),
                a.z() * b.x() - a.x() * b.z(),
                a.x() * b.y() - a.y() * b.x();

            for (int k = 0; k < 3; k++)
            {
                H[k](j, i) = h(k);
                H[k](i, j) = h(k);
            }
        }

    Matrix<double, 10, 1> Jm;

    for (int i = 0; i < 10; i++)
    {
        Matrix<double, 3, 10> H0;
        for (int a = 0; a < 3; a++)
            for (int b = 0; b < 10; b++)
                H0(a, b) = H[a](b, i);

        Eigen::Matrix<double, 3, 3> C = J_trans * H0.transpose();

        Eigen::Map<Eigen::VectorXd> C_reshape(C.data(), C.size());
        Eigen::Map<Eigen::VectorXd> B_reshape(B.data(), B.size());
        Jm(i) = manipulability * C_reshape.transpose() * B_reshape;
    }
    return Jm;
}

Eigen::Matrix<double, 4, 4> Car_arm::Fkine_collision(const std::vector<std::vector<double>> &dh_list,
                                                     const Eigen::Matrix<double, 10, 1> &q, bool isleft)
{
    if (size(dh_list) != 10)
    {
        std::cerr << "dimension error" << std::endl;
    }
    Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

    for (size_t i = 0; i < dh_list.size(); ++i)
    {
        double alpha = dh_list[i][0];
        double a = dh_list[i][1];
        double d = dh_list[i][2];
        double theta = dh_list[i][3] + q(i);

        Eigen::Matrix<double, 4, 4> A;
        A << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;

        T = T * A;

        // 用来 求  碰撞的 东西 ；
        if (isleft)
        {
            if (i == 1)
            {
                Eigen::Matrix<double, 4, 1> ball_down, ball_up;
                ball_down << 0, 0, 0, 1;
                ball_up << 0, -0.2, 0, 1;
                ball_up = T * ball_up;
                ball_down = T * ball_down;
                T2_ball1 = ball_down.block(0, 0, 3, 1);
                T2_ball2 = ball_up.block(0, 0, 3, 1);
            }
            else if (i == 2)
            {
                Eigen::Matrix<double, 4, 1> ball1, ball2, ball3, ball4;
                ball1 << 0, -0.1, 0, 1;
                ball2 << 0, 0.1, 0, 1;
                ball3 << 0, -0.1, -0.25, 1;
                ball4 << 0, 0.1, -0.25, 1;

                ball1 = T * ball1;
                ball2 = T * ball2;
                ball3 = T * ball3;
                ball4 = T * ball4;

                T3_ball1 = ball1.block(0, 0, 3, 1);
                T3_ball2 = ball2.block(0, 0, 3, 1);
                T3_ball3 = ball3.block(0, 0, 3, 1);
                T3_ball4 = ball4.block(0, 0, 3, 1);
            }

            else if (i == 6)
            {

                Eigen::Matrix<double, 4, 1> ball1, ball2;
                ball1 << -0.02, 0, 0, 1;
                ball2 << -0.02, 0.3, 0, 1;
                ball1 = T * ball1;
                ball2 = T * ball2;
                left_ball1 = ball1.block(0, 0, 3, 1);
                left_ball2 = ball2.block(0, 0, 3, 1);
            }
        }
        else // isleft ==  0
        {

            if (i == 6)
            {

                Eigen::Matrix<double, 4, 1> ball1, ball2;
                ball1 << -0.02, 0, 0, 1;
                ball2 << -0.02, -0.3, 0, 1;
                ball1 = T * ball1;
                ball2 = T * ball2;
                right_ball1 = ball1.block(0, 0, 3, 1);
                right_ball2 = ball2.block(0, 0, 3, 1);
            }
        }

        // 加入这个 重力 约束 ；
    }

    return T;
}

// 这个是检测 现在的 碰撞 距离
void Car_arm::dis_collision_self(double &dis_left_right, double &dis_left_T3, double &dis_left_T2,
                                 double &dis_right_T3, double &dis_right_T2)
{
    gkPolytope L, R, T2, T3;
    gkSimplex S;

    // 多面体L
    L.numpoints = 2;
    L.coord = new gkFloat *[2];
    L.coord[0] = new gkFloat[3]{left_ball1[0], left_ball1[1], left_ball1[2]};
    L.coord[1] = new gkFloat[3]{left_ball2[0], left_ball2[1], left_ball2[2]};

    // 初始化多面体 R
    R.numpoints = 2;
    R.coord = new gkFloat *[2];
    R.coord[0] = new gkFloat[3]{right_ball1[0], right_ball1[1], right_ball1[2]};
    R.coord[1] = new gkFloat[3]{right_ball2[0], right_ball2[1], right_ball2[2]};

    // 多面体 T2
    T2.numpoints = 2;
    T2.coord = new gkFloat *[2];
    T2.coord[0] = new gkFloat[3]{T2_ball1[0], T2_ball1[1], T2_ball1[2]};
    T2.coord[1] = new gkFloat[3]{T2_ball2[0], T2_ball2[1], T2_ball2[2]};

    // 多面体 T3
    T3.numpoints = 4;
    T3.coord = new gkFloat *[4];
    T3.coord[0] = new gkFloat[3]{T3_ball1[0], T3_ball1[1], T3_ball1[2]};
    T3.coord[1] = new gkFloat[3]{T3_ball2[0], T3_ball2[1], T3_ball2[2]};
    T3.coord[2] = new gkFloat[3]{T3_ball3[0], T3_ball3[1], T3_ball3[2]};
    T3.coord[3] = new gkFloat[3]{T3_ball4[0], T3_ball4[1], T3_ball4[2]};

    dis_left_right = compute_minimum_distance(L, R, &S);
    dis_left_T2 = compute_minimum_distance(L, T2, &S);
    dis_left_T3 = compute_minimum_distance(L, T3, &S);
    dis_right_T2 = compute_minimum_distance(R, T2, &S);
    dis_right_T3 = compute_minimum_distance(R, T3, &S);

    dis_left_right -= 0.05;
    dis_left_T2 -= 0.2;
    dis_left_T3 -= 0.2;
    dis_right_T2 -= 0.2;
    dis_right_T3 -= 0.2;
}

// 这个函数的作用是 根据给定的 q_all 17 个关节角度来 计算距离  ，用来之后差分
void Car_arm::dis_collision_self(Eigen::Matrix<double, 17, 1> q, double &dis_left_right, double &dis_left_T3, double &dis_left_T2,
                                 double &dis_right_T3, double &dis_right_T2)
{
    Eigen::Matrix<double, 3, 1> left_ball1, left_ball2,
        right_ball1, right_ball2,
        T3_ball1, T3_ball2, T3_ball3, T3_ball4,
        T2_ball1, T2_ball2;

    auto calculate_T = [](const std::vector<double> &dh, double q)
    {
        double alpha = dh[0];
        double a = dh[1];
        double d = dh[2];
        double theta = dh[3] + q;

        Eigen::Matrix<double, 4, 4> A;
        A << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;
        return A;
    };

    Eigen::Matrix<double, 4, 4> body2 = calculate_T(dh_list_Waist[0], q[0]) * calculate_T(dh_list_Waist[1], q[1]);
    {
        Eigen::Matrix<double, 4, 1> ball_down, ball_up;
        ball_down << 0, 0, 0, 1;
        ball_up << 0, -0.2, 0, 1;
        ball_up = body2 * ball_up;
        ball_down = body2 * ball_down;
        T2_ball1 = ball_down.block(0, 0, 3, 1);
        T2_ball2 = ball_up.block(0, 0, 3, 1);
    }
    Eigen::Matrix<double, 4, 4> body3 = body2 * calculate_T(dh_list_Waist[2], q[2]);
    {
        Eigen::Matrix<double, 4, 1> ball1, ball2, ball3, ball4;
        ball1 << 0, -0.1, 0, 1;
        ball2 << 0, 0.1, 0, 1;
        ball3 << 0, -0.1, -0.25, 1;
        ball4 << 0, 0.1, -0.25, 1;

        ball1 = body3 * ball1;
        ball2 = body3 * ball2;
        ball3 = body3 * ball3;
        ball4 = body3 * ball4;

        T3_ball1 = ball1.block(0, 0, 3, 1);
        T3_ball2 = ball2.block(0, 0, 3, 1);
        T3_ball3 = ball3.block(0, 0, 3, 1);
        T3_ball4 = ball4.block(0, 0, 3, 1);
    }

    Eigen::Matrix<double, 4, 4> T_left = body3 * calculate_T(dh_list_Lbase_Leffector[0], q[3]) * calculate_T(dh_list_Lbase_Leffector[1], q[4]) * calculate_T(dh_list_Lbase_Leffector[2], q[5]) * calculate_T(dh_list_Lbase_Leffector[3], q[6]);

    {
        Eigen::Matrix<double, 4, 1> ball1, ball2;
        ball1 << -0.02, 0, 0, 1;
        ball2 << -0.02, 0.3, 0, 1;
        ball1 = T_left * ball1;
        ball2 = T_left * ball2;
        left_ball1 = ball1.block(0, 0, 3, 1);
        left_ball2 = ball2.block(0, 0, 3, 1);
    }

    Eigen::Matrix<double, 4, 4> T_right = body3 * calculate_T(dh_list_Rbase_Reffector[0], q[10]) * calculate_T(dh_list_Rbase_Reffector[1], q[11]) * calculate_T(dh_list_Rbase_Reffector[2], q[12]) * calculate_T(dh_list_Rbase_Reffector[3], q[13]);
    {
        Eigen::Matrix<double, 4, 1> ball1, ball2;
        ball1 << -0.02, 0, 0, 1;
        ball2 << -0.02, -0.3, 0, 1;
        ball1 = T_right * ball1;
        ball2 = T_right * ball2;
        right_ball1 = ball1.block(0, 0, 3, 1);
        right_ball2 = ball2.block(0, 0, 3, 1);
    }

    gkPolytope L, R, T2, T3;
    gkSimplex S;

    // 多面体L
    L.numpoints = 2;
    L.coord = new gkFloat *[2];
    L.coord[0] = new gkFloat[3]{left_ball1[0], left_ball1[1], left_ball1[2]};
    L.coord[1] = new gkFloat[3]{left_ball2[0], left_ball2[1], left_ball2[2]};

    // 初始化多面体 R
    R.numpoints = 2;
    R.coord = new gkFloat *[2];
    R.coord[0] = new gkFloat[3]{right_ball1[0], right_ball1[1], right_ball1[2]};
    R.coord[1] = new gkFloat[3]{right_ball2[0], right_ball2[1], right_ball2[2]};

    // 多面体 T2
    T2.numpoints = 2;
    T2.coord = new gkFloat *[2];
    T2.coord[0] = new gkFloat[3]{T2_ball1[0], T2_ball1[1], T2_ball1[2]};
    T2.coord[1] = new gkFloat[3]{T2_ball2[0], T2_ball2[1], T2_ball2[2]};

    // 多面体 T3
    T3.numpoints = 4;
    T3.coord = new gkFloat *[4];
    T3.coord[0] = new gkFloat[3]{T3_ball1[0], T3_ball1[1], T3_ball1[2]};
    T3.coord[1] = new gkFloat[3]{T3_ball2[0], T3_ball2[1], T3_ball2[2]};
    T3.coord[2] = new gkFloat[3]{T3_ball3[0], T3_ball3[1], T3_ball3[2]};
    T3.coord[3] = new gkFloat[3]{T3_ball4[0], T3_ball4[1], T3_ball4[2]};

    dis_left_right = compute_minimum_distance(L, R, &S);
    dis_left_T2 = compute_minimum_distance(L, T2, &S);
    dis_left_T3 = compute_minimum_distance(L, T3, &S);
    dis_right_T2 = compute_minimum_distance(R, T2, &S);
    dis_right_T3 = compute_minimum_distance(R, T3, &S);

    dis_left_right -= 0.05;
    dis_left_T2 -= 0.2;
    dis_left_T3 -= 0.2;
    dis_right_T2 -= 0.2;
    dis_right_T3 -= 0.2;
}

Eigen::Matrix<double, 2, 1> Car_arm::ZMP(Eigen::Matrix<double, 17, 1> &q,
                                         Eigen::Matrix<double, 17, 1> &dq, Eigen::Matrix<double, 17, 1> &ddq)
{
    Eigen::Matrix<double, 3, 1> q_waist = q.block(0, 0, 3, 1);
    Eigen::Matrix<double, 7, 1> q_left = q.block(3, 0, 7, 1);
    Eigen::Matrix<double, 7, 1> q_right = q.block(10, 0, 7, 1);

    Eigen::Matrix<double, 3, 1> dq_waist = dq.block(0, 0, 3, 1);
    Eigen::Matrix<double, 7, 1> dq_left = dq.block(3, 0, 7, 1);
    Eigen::Matrix<double, 7, 1> dq_right = dq.block(10, 0, 7, 1);

    Eigen::Matrix<double, 3, 1> ddq_waist = ddq.block(0, 0, 3, 1);
    Eigen::Matrix<double, 7, 1> ddq_left = ddq.block(3, 0, 7, 1);
    Eigen::Matrix<double, 7, 1> ddq_right = ddq.block(10, 0, 7, 1);

    array<Eigen::Matrix<double, 3, 3>, 3> R_Waist; // R_i_i-1  i-1 是 base
    array<Eigen::Matrix<double, 3, 3>, 7> R_Left;
    array<Eigen::Matrix<double, 3, 3>, 7> R_Right;

    array<Eigen::Matrix<double, 3, 1>, 3> P_Waist; // Pi 在 i-1  ；
    array<Eigen::Matrix<double, 3, 1>, 7> P_Left;
    array<Eigen::Matrix<double, 3, 1>, 7> P_Right;

    array<Eigen::Matrix<double, 3, 1>, 3> Pc_Waist = Waist_MassCenter; // P ci 在 i 中
    array<Eigen::Matrix<double, 3, 1>, 7> Pc_Left = Left_MassCenter;
    array<Eigen::Matrix<double, 3, 1>, 7> Pc_Right = Right_MassCenter;

    //--------------------------------------- 求 各个 R ------------------------------------------
    for (int i = 0; i < 3; i++)
    {
        double alpha = dh_list_Waist[i][0];
        double a = dh_list_Waist[i][1];
        double d = dh_list_Waist[i][2];
        double theta = dh_list_Waist[i][3] + q_waist(i);

        Eigen::Matrix<double, 4, 4> T; // T_i_i+1
        T << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;
        R_Waist[i] = T.block(0, 0, 3, 3).transpose();
        P_Waist[i] = T.block(0, 3, 3, 1);
    }

    for (int i = 0; i < 7; i++)
    {
        double alpha = dh_list_Lbase_Leffector[i][0];
        double a = dh_list_Lbase_Leffector[i][1];
        double d = dh_list_Lbase_Leffector[i][2];
        double theta = dh_list_Lbase_Leffector[i][3] + q_left(i);

        Eigen::Matrix<double, 4, 4> T; // T_i_i+1
        T << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;
        R_Left[i] = T.block(0, 0, 3, 3).transpose();
        P_Left[i] = T.block(0, 3, 3, 1);
    }

    for (int i = 0; i < 7; i++)
    {
        double alpha = dh_list_Rbase_Reffector[i][0];
        double a = dh_list_Rbase_Reffector[i][1];
        double d = dh_list_Rbase_Reffector[i][2];
        double theta = dh_list_Rbase_Reffector[i][3] + q_right(i);

        Eigen::Matrix<double, 4, 4> T; // T_i_i+1
        T << cos(theta), -sin(theta), 0, a,
            sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
            sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha),
            0, 0, 0, 1;
        R_Right[i] = T.block(0, 0, 3, 3).transpose();
        P_Right[i] = T.block(0, 3, 3, 1);
    }

    //--------------------------------外推-------------------------------------
    array<Eigen::Matrix<double, 3, 1>, 3> w_Waist, dw_Waist; // w _i _i
    array<Eigen::Matrix<double, 3, 1>, 7> w_Left, dw_Left;
    array<Eigen::Matrix<double, 3, 1>, 7> w_Right, dw_Right;

    array<Eigen::Matrix<double, 3, 1>, 3> v_Waist, dv_Waist; // v_i _ i
    array<Eigen::Matrix<double, 3, 1>, 7> v_Left, dv_Left;
    array<Eigen::Matrix<double, 3, 1>, 7> v_Right, dv_Right;

    array<Eigen::Matrix<double, 3, 1>, 3> dvc_Waist;
    array<Eigen::Matrix<double, 3, 1>, 7> dvc_Left;
    array<Eigen::Matrix<double, 3, 1>, 7> dvc_Right;

    array<Eigen::Matrix<double, 3, 1>, 3> F_Waist, N_Waist, f_Waist, n_Waist; // F _i _i N_i_i
    array<Eigen::Matrix<double, 3, 1>, 7> F_Left, N_Left, f_Left, n_Left;
    array<Eigen::Matrix<double, 3, 1>, 7> F_Right, N_Right, f_Right, n_Right;

    array<double, 7> tao_Left, tao_Right;
    array<double, 3> tao_Waist;

    // -waist
    for (int i = 0; i < 3; i++)
    {
        if (i == 0)
        {
            w_Waist[i] = R_Waist[i] * Eigen::Matrix<double, 3, 1>::Zero() + dq_waist[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dw_Waist[i] = R_Waist[i] * Eigen::Matrix<double, 3, 1>::Zero() + (R_Waist[i] * Eigen::Matrix<double, 3, 1>::Zero()).cross(dq_waist[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished()) + ddq_waist[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dv_Waist[i] = R_Waist[i] * (Eigen::Matrix<double, 3, 1>::Zero().cross(P_Waist[i]) + Eigen::Matrix<double, 3, 1>::Zero().cross(Eigen::Matrix<double, 3, 1>::Zero().cross(P_Waist[i])) + (Eigen::Matrix<double, 3, 1>() << 9.8, 0, 0).finished()); // 加入了 重力
            dvc_Waist[i] = dw_Waist[i].cross(Pc_Waist[i]) + w_Waist[i].cross(w_Waist[i].cross(Pc_Waist[i])) + dv_Waist[i];
            F_Waist[i] = Waist_Mass[i] * dvc_Waist[i];
            N_Waist[i] = Waist_Inertia[i] * dw_Waist[i] + w_Waist[i].cross(Waist_Inertia[i] * w_Waist[i]);
        }
        else
        {
            w_Waist[i] = R_Waist[i] * w_Waist[i - 1] + dq_waist[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dw_Waist[i] = R_Waist[i] * dw_Waist[i - 1] + (R_Waist[i] * dw_Waist[i - 1]).cross(dq_waist[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished()) + ddq_waist[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dv_Waist[i] = R_Waist[i] * (dw_Waist[i - 1].cross(P_Waist[i]) + w_Waist[i - 1].cross(w_Waist[i - 1].cross(P_Waist[i])) + dv_Waist[i - 1]);
            dvc_Waist[i] = dw_Waist[i].cross(Pc_Waist[i]) + w_Waist[i].cross(w_Waist[i].cross(Pc_Waist[i])) + dv_Waist[i];
            F_Waist[i] = Waist_Mass[i] * dvc_Waist[i];
            N_Waist[i] = Waist_Inertia[i] * dw_Waist[i] + w_Waist[i].cross(Waist_Inertia[i] * w_Waist[i]);
        }
    }

    // left arm
    for (int i = 0; i < 7; i++)
    {
        if (i == 0)
        {
            w_Left[i] = R_Left[i] * w_Waist[2] + dq_left[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dw_Left[i] = R_Left[i] * dw_Waist[2] + (R_Left[i] * dw_Waist[2]).cross(dq_left[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished()) + ddq_left[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dv_Left[i] = R_Left[i] * (dw_Waist[2].cross(P_Left[i]) + w_Waist[2].cross(w_Waist[2].cross(P_Left[i])) + dv_Waist[2]);
            dvc_Left[i] = dw_Left[i].cross(Pc_Left[i]) + w_Left[i].cross(w_Left[i].cross(Pc_Left[i])) + dv_Left[i];
            F_Left[i] = Left_Mass[i] * dvc_Left[i];
            N_Left[i] = Left_Inertia[i] * dw_Left[i] + w_Left[i].cross(Left_Inertia[i] * w_Left[i]);
        }
        else
        {
            w_Left[i] = R_Left[i] * w_Left[i - 1] + dq_left[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dw_Left[i] = R_Left[i] * dw_Left[i - 1] + (R_Left[i] * dw_Left[i - 1]).cross(dq_left[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished()) + ddq_left[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dv_Left[i] = R_Left[i] * (dw_Left[i - 1].cross(P_Left[i]) + w_Left[i - 1].cross(w_Left[i - 1].cross(P_Left[i])) + dv_Left[i - 1]);
            dvc_Left[i] = dw_Left[i].cross(Pc_Left[i]) + w_Left[i].cross(w_Left[i].cross(Pc_Left[i])) + dv_Left[i];
            F_Left[i] = Left_Mass[i] * dvc_Left[i];
            N_Left[i] = Left_Inertia[i] * dw_Left[i] + w_Left[i].cross(Left_Inertia[i] * w_Left[i]);
        }
    }

    // right_arm
    for (int i = 0; i < 7; i++)
    {
        if (i == 0)
        {
            w_Right[i] = R_Right[i] * w_Waist[2] + dq_right[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dw_Right[i] = R_Right[i] * dw_Waist[2] + (R_Right[i] * dw_Waist[2]).cross(dq_right[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished()) + ddq_right[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dv_Right[i] = R_Right[i] * (dw_Waist[2].cross(P_Right[i]) + w_Waist[2].cross(w_Waist[2].cross(P_Right[i])) + dv_Waist[2]);
            dvc_Right[i] = dw_Right[i].cross(Pc_Right[i]) + w_Right[i].cross(w_Right[i].cross(Pc_Right[i])) + dv_Right[i];
            F_Right[i] = Right_Mass[i] * dvc_Right[i];
            N_Right[i] = Right_Inertia[i] * dw_Right[i] + w_Right[i].cross(Right_Inertia[i] * w_Right[i]);
        }
        else
        {
            w_Right[i] = R_Right[i] * w_Right[i - 1] + dq_right[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dw_Right[i] = R_Right[i] * dw_Right[i - 1] + (R_Right[i] * dw_Right[i - 1]).cross(dq_right[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished()) + ddq_right[i] * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();
            dv_Right[i] = R_Right[i] * (dw_Right[i - 1].cross(P_Right[i]) + w_Right[i - 1].cross(w_Right[i - 1].cross(P_Right[i])) + dv_Right[i - 1]);
            dvc_Right[i] = dw_Right[i].cross(Pc_Right[i]) + w_Right[i].cross(w_Right[i].cross(Pc_Right[i])) + dv_Right[i];
            F_Right[i] = Right_Mass[i] * dvc_Right[i];
            N_Right[i] = Right_Inertia[i] * dw_Right[i] + w_Right[i].cross(Right_Inertia[i] * w_Right[i]);
        }
    }

    // ------------------------------attention ！！！！！！！！！！！！！！！！！！！！！！！！！-----------------------------------------
    // 这里留了两个 接口 ， f_external 在 6系中的表示  和 n_external 在 6系中的表示
    Eigen::Matrix<double, 3, 1> f_left_external, n_left_external, f_right_external, n_right_external;
    f_left_external.setZero();
    n_left_external.setZero();
    f_right_external.setZero();
    n_right_external.setZero();
    // -----------------------------------------------------------------------------内推------------------------------------------------------------------------------------------------------
    for (int i = 6; i >= 0; i--)
    {
        if (i == 6)
        {
            f_Left[i] = f_left_external + F_Left[i];
            n_Left[i] = N_Left[i] + n_left_external + Pc_Left[i].cross(F_Left[i]); // + P_Left[i + 1].cross(R_Left[i + 1].transpose() * f_Left[i + 1]);
            tao_Left[i] = (n_Left[i].transpose() * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished())(0);
        }
        else
        {
            f_Left[i] = R_Left[i + 1].transpose() * f_Left[i + 1] + F_Left[i];
            n_Left[i] = N_Left[i] + R_Left[i + 1].transpose() * n_Left[i + 1] + Pc_Left[i].cross(F_Left[i]) + P_Left[i + 1].cross(R_Left[i + 1].transpose() * f_Left[i + 1]);
            tao_Left[i] = (n_Left[i].transpose() * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished())(0);
        }
    }

    for (int i = 6; i >= 0; i--)
    {
        if (i == 6)
        {
            f_Right[i] = f_right_external + F_Right[i];
            n_Right[i] = N_Right[i] + n_right_external + Pc_Right[i].cross(F_Right[i]); // + P_Right[i + 1].cross(R_Right[i + 1].transpose() * f_Right[i + 1]);
            tao_Right[i] = (n_Right[i].transpose() * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished())(0);
        }
        else
        {
            f_Right[i] = R_Right[i + 1].transpose() * f_Right[i + 1] + F_Right[i];
            n_Right[i] = N_Right[i] + R_Right[i + 1].transpose() * n_Right[i + 1] + Pc_Right[i].cross(F_Right[i]) + P_Right[i + 1].cross(R_Right[i + 1].transpose() * f_Right[i + 1]);
            tao_Right[i] = (n_Right[i].transpose() * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished())(0);
        }
    }

    for (int i = 2; i >= 0; i--)
    {
        if (i == 2)
        {
            f_Waist[i] = R_Left[0].transpose() * f_Left[0] + R_Right[0].transpose() * f_Right[0] + F_Waist[i];
            n_Waist[i] = N_Waist[i] + R_Left[0].transpose() * n_Left[0] + R_Right[0].transpose() * n_Right[0] + Pc_Waist[i].cross(F_Waist[i]) + P_Left[0].cross(R_Left[0].transpose() * f_Left[0]) + P_Right[0].cross(R_Right[0].transpose() * f_Right[0]);
            tao_Waist[i] = (n_Waist[i].transpose() * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished())(0);
        }
        else
        {

            f_Waist[i] = R_Waist[i + 1].transpose() * f_Waist[i + 1] + F_Waist[i];
            n_Waist[i] = N_Waist[i] + R_Waist[i + 1].transpose() * n_Waist[i + 1] + Pc_Waist[i].cross(F_Waist[i]) + P_Waist[i + 1].cross(R_Waist[i + 1].transpose() * f_Waist[i + 1]);
            tao_Waist[i] = (n_Waist[i].transpose() * (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished())(0);
        }
    }

    // f n 是对底盘的 力 和 力矩
    Eigen::Matrix<double, 3, 1> f = -f_Waist[0], n = -n_Waist[0];

    // 先变到 base 坐标系
    f = R_Waist[0].transpose() * f;
    n = R_Waist[0].transpose() * n;

    Eigen::Matrix<double, 3, 3> R_car_base;
    R_car_base = T_Carbase_Base.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> P_car_base = T_Carbase_Base.block(0, 3, 3, 1);

    // 伴随变换（Adjoint）实现坐标系变换
    Eigen::Matrix<double, 6, 6> Adjoint;
    Adjoint.block(0, 0, 3, 3) = R_car_base;
    Adjoint.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Zero();
    Adjoint.block(3, 0., 3, 3) = (Eigen::Matrix<double, 3, 3>() << 0, -P_car_base(2), P_car_base(1),
                                  P_car_base(2), 0, -P_car_base(0),
                                  -P_car_base(1), P_car_base(0), 0)
                                     .finished() *
                                 R_car_base;
    Adjoint.block(3, 3, 3, 3) = R_car_base;

    Eigen::Matrix<double, 6, 1> w;
    w << n, f;
    w = Adjoint * w;
    double fz = w(5), wx = w(0), wy = w(1);
    Eigen::Matrix<double, 2, 1> zmpxy;

    // 注意 fz 是body 给车的力 ， 应该取反
    zmpxy << -wy / fz, wx / fz;
    return zmpxy;
    //
}
