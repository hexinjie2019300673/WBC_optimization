#ifndef VARIABLE_MPC
#define VARIABLE_MPC

#include <iostream>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <vector>
#include <cmath>
#include <OsqpEigen/OsqpEigen.h>
#include <osqp.h>
#include "openGJK.h"

using namespace std;
using namespace Eigen;
#define M_PI 3.14159265358979323846
class Car_arm
{
public:
    Eigen::Matrix<double, 7, 1> q_left, q_right, q_min, q_max;
    Eigen::Matrix<double, 3, 1> q_waist, q_waist_min, q_waist_max;
    Eigen::Matrix<double, 10, 1> q_Base_Leffector, q_Base_Reffector;

    Eigen::Matrix<double, 2, 1> q_car, dq_car;

    Eigen::Matrix<double, 6, 1> slack_left, slack_right, Velocity_World_leftstar, Velocity_World_rightstar;

    std::vector<std::vector<double>> dh_list_Lbase_Leffector, dh_list_Rbase_Reffector, dh_list_Waist, dh_list_Base_Leffector, dh_list_Base_Reffector;
    Eigen::Matrix<double, 4, 4> T_Carbase_Base;
    double ka, k_episilon, beta, eta, ps, pi;
    double dis_Carbase_Eleftstar, dis_Carbase_Erightstar, dis_Leffector_Eleftstar, dis_Reffector_Erightstar, theta_Carbase_Leffector, theta_Carbase_Reffector;

    Eigen::Matrix<double, 4, 4> T_World_Eleftstar, T_World_Erightstar, T_World_Carbase;

    Eigen::Matrix<double, 4, 4> T_World_Leffector, T_World_Reffector;
    Eigen::Matrix<double, 4, 4> T_Base_Leffector, T_Base_Reffector;

    // 末端执行器相对于link7的齐次矩阵
    Eigen::Matrix<double, 4, 4> T_leftend_tool, T_rightend_tool;

    // 与目标的距离
    double dis_left_leftstar, dis_right_rightstar;
    // 自碰撞 部分的信息
    double r_collision_self;
    double dis_left_T2, dis_left_T3, dis_right_T2, dis_right_T3, dis_left_right;
    Eigen::Matrix<double, 3, 1> left_ball1, left_ball2,
        right_ball1, right_ball2,
        T3_ball1, T3_ball2, T3_ball3, T3_ball4,
        T2_ball1, T2_ball2;

    // 外部碰撞
    vector<array<double, 3>> outer_body(); // 这个里面装的 是一个 碰撞体 ，碰撞体 里面 装很多点 ；
    double out_distance;

    // 动力学参数
    array<double, 3> Waist_Mass;
    array<double, 7> Left_Mass;
    array<double, 7> Right_Mass;
    array<Eigen::Matrix<double, 3, 1>, 3> Waist_MassCenter;
    array<Eigen::Matrix<double, 3, 1>, 7> Left_MassCenter;
    array<Eigen::Matrix<double, 3, 1>, 7> Right_MassCenter;
    array<Eigen::Matrix<double, 3, 3>, 3> Waist_Inertia;
    array<Eigen::Matrix<double, 3, 3>, 7> Left_Inertia;
    array<Eigen::Matrix<double, 3, 3>, 7> Right_Inertia;

    /*
    * @brief 构造函数，完成DH等参数的初始化
    */
    Car_arm();

    /*
     * @brief 析构函数，暂时没使用
     */
    ~Car_arm() {}

    void Update_parameter_single(const Eigen::Matrix<double, 4, 4> &T_World_Carbase_input,
                                 const Eigen::Matrix<double, 4, 4> &T_World_Estar_input,
                                 const Eigen::Matrix<double, 3, 1> &q_waist_input,
                                 const Eigen::Matrix<double, 7, 1> &q_left_input,
                                 const Eigen::Matrix<double, 7, 1> &q_right_input);
    Eigen::Matrix<double, 9, 1> Solver_single();

    void Update_parameter_double(const Eigen::Matrix<double, 4, 4> &T_World_Carbase_input,
                                 const Eigen::Matrix<double, 4, 4> &T_World_Eleftstar_input,
                                 const Eigen::Matrix<double, 4, 4> &T_World_Erightstar_input,
                                 const Eigen::Matrix<double, 3, 1> &q_waist_input,
                                 const Eigen::Matrix<double, 7, 1> &q_left_input,
                                 const Eigen::Matrix<double, 7, 1> &q_right_input);
    Eigen::Matrix<double, 19, 1> Solver_double();

    void Update_parameter_double_velocity(const Eigen::Matrix<double, 4, 4> &T_World_Carbase_input,
                                          const Eigen::Matrix<double, 4, 4> &T_World_Eleftstar_input,
                                          const Eigen::Matrix<double, 4, 4> &T_World_Erightstar_input,
                                          const Eigen::Matrix<double, 6, 1> &Velocity_World_Eleftstar_input,
                                          const Eigen::Matrix<double, 6, 1> &Velocity_World_Erightstar_input,
                                          const Eigen::Matrix<double, 3, 1> &q_waist_input,
                                          const Eigen::Matrix<double, 7, 1> &q_left_input,
                                          const Eigen::Matrix<double, 7, 1> &q_right_input);

    Eigen::Matrix<double, 19, 1> Solver_double_velocity();

    void Update_parameter_collision(const Eigen::Matrix<double, 4, 4> &T_World_Carbase_input,
                                    const Eigen::Matrix<double, 4, 4> &T_World_Eleftstar_input,
                                    const Eigen::Matrix<double, 4, 4> &T_World_Erightstar_input,
                                    const Eigen::Matrix<double, 6, 1> &Velocity_World_Eleftstar_input,
                                    const Eigen::Matrix<double, 6, 1> &Velocity_World_Erightstar_input,
                                    const Eigen::Matrix<double, 3, 1> &q_waist_input,
                                    const Eigen::Matrix<double, 7, 1> &q_left_input,
                                    const Eigen::Matrix<double, 7, 1> &q_right_input);

    Eigen::Matrix<double, 19, 1> Solver_collision();

    void Update_parameter_balance(const Eigen::Matrix<double, 4, 4> &T_World_Carbase_input,
                                  const Eigen::Matrix<double, 4, 4> &T_World_Eleftstar_input,
                                  const Eigen::Matrix<double, 4, 4> &T_World_Erightstar_input,
                                  const Eigen::Matrix<double, 6, 1> &Velocity_World_Eleftstar_input,
                                  const Eigen::Matrix<double, 6, 1> &Velocity_World_Erightstar_input,
                                  const Eigen::Matrix<double, 3, 1> &q_waist_input,
                                  const Eigen::Matrix<double, 7, 1> &q_left_input,
                                  const Eigen::Matrix<double, 7, 1> &q_right_input);

    Eigen::Matrix<double, 19, 1> Solver_balance();

    void Update_parameter_test(const Eigen::Matrix<double, 4, 4> &T_World_Carbase_input,
                               const Eigen::Matrix<double, 4, 4> &T_World_Eleftstar_input,
                               const Eigen::Matrix<double, 4, 4> &T_World_Erightstar_input,
                               const Eigen::Matrix<double, 6, 1> &Velocity_World_Eleftstar_input,
                               const Eigen::Matrix<double, 6, 1> &Velocity_World_Erightstar_input,
                               const Eigen::Matrix<double, 3, 1> &q_waist_input,
                               const Eigen::Matrix<double, 7, 1> &q_left_input,
                               const Eigen::Matrix<double, 7, 1> &q_right_input);

    Eigen::Matrix<double, 19, 1> Solver_test();

    /*
     * @brief 更新求解所需的实时参数
     * @param 世界坐标系下底盘位置、左执行器目标位置、右执行器目标位置、左执行器目标速度、右执行器目标速度、车的腰关节角度、左臂关节角度、右臂关节角度
     */
    void Update_parameter(const Eigen::Matrix<double, 4, 4> &T_World_Carbase_input,
                          const Eigen::Matrix<double, 4, 4> &T_World_Eleftstar_input,
                          const Eigen::Matrix<double, 4, 4> &T_World_Erightstar_input,
                          const Eigen::Matrix<double, 6, 1> &Velocity_World_Eleftstar_input,
                          const Eigen::Matrix<double, 6, 1> &Velocity_World_Erightstar_input,
                          const Eigen::Matrix<double, 3, 1> &q_waist_input,
                          const Eigen::Matrix<double, 7, 1> &q_left_input,
                          const Eigen::Matrix<double, 7, 1> &q_right_input);

    /*
     * @brief 更新参数求解机器人运动的速度指令
     * @return 机器人19*1 的运动指令 ， 2底盘 + 3个腰关节 + 7个左臂 + 7 个右臂
     */
    Eigen::Matrix<double, 19, 1> Solver();

    /*
     * @brief 更新参数求解机器人运动的速度指令，没有考虑跟踪轨迹或最终最后目标
     * @return 机器人19*1 的运动指令 ， 2底盘 + 3个腰关节 + 7个左臂 + 7 个右臂
     */
    Eigen::Matrix<double, 19, 1> Solver2();

    /*
     * @brief 求解正运动学，求出dh——list中最后元素对应坐标系 相对于第一个元素的坐标系的 齐次矩阵， 该函数有多个重载
     * @param DH参数，关节角度q
     * @return 4*4 的 其次矩阵
     */
    Eigen::Matrix<double, 4, 4> Fkine(const std::vector<std::vector<double>> &dh_list, const Eigen::Matrix<double, 10, 1> &q);

    Eigen::Matrix<double, 4, 4> Fkine_collision(const std::vector<std::vector<double>> &dh_list,
                                                const Eigen::Matrix<double, 10, 1> &q, bool isleft);

    Eigen::Matrix<double, 4, 4> Fkine(const std::vector<std::vector<double>> &dh_list, const Eigen::Matrix<double, 7, 1> &q);
    Eigen::Matrix<double, 4, 4> Fkine(const std::vector<std::vector<double>> &dh_list, const Eigen::Matrix<double, 3, 1> &q);

    /*
     * @brief 求解雅可比矩阵 末端速度相对于 dh的基坐标系，有多个重载
     * @param DH参数，关节角度q
     * @return 6*n矩阵
     */
    Eigen::Matrix<double, 6, 10> Jacobian(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 10, 1> &q);
    Eigen::Matrix<double, 6, 7> Jacobian(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 7, 1> &q);
    Eigen::Matrix<double, 6, 3> Jacobian(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 3, 1> &q);

    /*
     * @brief 求解可操作度雅可比矩阵 有多个重载
     * @param DH参数，关节角度q
     * @return 6*n矩阵
     */
    Eigen::Matrix<double, 3, 1> Manipulability(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 3, 1> &q);

    Eigen::Matrix<double, 7, 1> Manipulability(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 7, 1> &q);
    Eigen::Matrix<double, 10, 1> Manipulability(const vector<vector<double>> &dh_list, const Eigen::Matrix<double, 10, 1> &q);

    /*
     * @brief 求解自碰撞的距离
     * @param 传入左臂右臂距离， 左臂关于腰第一个凸包的距离， 左臂关于腰第二个凸包的距离，右臂关于腰第一个凸包的距离， 右臂关于腰第二个凸包的距离
     */
    void dis_collision_self(double &dis_left_right, double &dis_left_T3, double &dis_left_T2, double &dis_right_T3, double &dis_right_T2);
    /*
     * @brief 求解自碰撞的距离
     * @param 传入腰+两臂17个关节角度， 左臂右臂距离， 左臂关于腰第一个凸包的距离， 左臂关于腰第二个凸包的距离，右臂关于腰第一个凸包的距离， 右臂关于腰第二个凸包的距离
     */
    void dis_collision_self(Eigen::Matrix<double, 17, 1> q, double &dis_left_right, double &dis_left_T3, double &dis_left_T2, double &dis_right_T3, double &dis_right_T2);
    Eigen::Matrix<double, 2, 1> ZMP(Eigen::Matrix<double, 17, 1> &q,
                                    Eigen::Matrix<double, 17, 1> &dq, Eigen::Matrix<double, 17, 1> &ddq);
};

#endif