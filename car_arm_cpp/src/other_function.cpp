#include "other_function.h"

void dis_collision_self(Eigen::Matrix<double, 17, 1> q, double &dis_left_right, double &dis_left_T3, double &dis_left_T2,
                        double &dis_right_T3, double &dis_right_T2)
{
    Eigen::Matrix<double, 3, 1> left_ball1, left_ball2,
        right_ball1, right_ball2,
        T3_ball1, T3_ball2, T3_ball3, T3_ball4,
        T2_ball1, T2_ball2;

    std::vector<std::vector<double>> dh_list_Lbase_Leffector = {
        {-M_PI / 2, 0, 0.2197, 0}, // 左臂
        {M_PI / 2, 0, 0, 0},
        {-M_PI / 2, 0, 0.3005, 0},
        {M_PI / 2, 0.0225, 0, 0},
        {-M_PI / 2, -0.0225, 0.25, 0},
        {M_PI / 2, 0, 0, M_PI / 2},
        {-M_PI / 2, 0, 0, 0},
    };

    std::vector<std::vector<double>> dh_list_Rbase_Reffector = {
        {M_PI / 2, 0, 0.2197, 0}, // 右臂
        {-M_PI / 2, 0, 0, 0},
        {M_PI / 2, 0, 0.3005, 0},
        {-M_PI / 2, 0.0225, 0, 0},
        {M_PI / 2, -0.0225, 0.25, 0},
        {-M_PI / 2, 0, 0, -M_PI / 2},
        {-M_PI / 2, 0, 0, 0},
    };

    std::vector<std::vector<double>> dh_list_Waist =
        {
            {0, 0, 0, 0}, // 躯干
            {0, 0.32, 0, M_PI / 2},
            {M_PI / 2, 0, 0.5909, 0},
        };

    std::vector<std::vector<double>> dh_list_Base_Leffector = {
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

    std::vector<std::vector<double>> dh_list_Base_Reffector = {
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