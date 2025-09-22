#include <iostream>
#include <Eigen/Dense>
#include "openGJK.h"
#include <vector>
void dis_collision_self(Eigen::Matrix<double, 17, 1> q, double &dis_left_right, double &dis_left_T3, double &dis_left_T2,
                        double &dis_right_T3, double &dis_right_T2);