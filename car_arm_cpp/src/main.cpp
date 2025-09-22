#include "car_arm.h"
#include <iostream>
#include <chrono>

/////////////////////////////////////////////////////////////////////////////single_arm//////////////////////////////////////////////////////////
/*
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

#define PORT 12345

int main()
{
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    double recv_data[9 + 3 + 9 + 3 + 3 + 7 + 7];
    double send_data[19];

    server_fd = socket(AF_INET, SOCK_STREAM, 0);

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 1);
    std::cout << "Server listening on port " << PORT << "...\n";

    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);

    // 开始设计
    Car_arm car_arm;
    Eigen::Matrix<double, 4, 4> T_World_Estar, T_World_Carbase;
    T_World_Estar = Eigen::Matrix<double, 4, 4>::Identity();
    T_World_Carbase = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 3, 1> q_waist;
    Eigen::Matrix<double, 7, 1> q_left, q_right;

    while (true)
    {
        int r = recv(new_socket, recv_data, sizeof(recv_data), 0);
        if (r <= 0)
            break;
        T_World_Carbase(0, 0) = recv_data[0];
        T_World_Carbase(0, 1) = recv_data[1];
        T_World_Carbase(0, 2) = recv_data[2];
        T_World_Carbase(1, 0) = recv_data[3];
        T_World_Carbase(1, 1) = recv_data[4];
        T_World_Carbase(1, 2) = recv_data[5];
        T_World_Carbase(2, 0) = recv_data[6];
        T_World_Carbase(2, 1) = recv_data[7];
        T_World_Carbase(2, 2) = recv_data[8];
        // 位置
        T_World_Carbase(0, 3) = recv_data[9];
        T_World_Carbase(1, 3) = recv_data[10];
        T_World_Carbase(2, 3) = recv_data[11];

        //
        T_World_Estar(0, 0) = recv_data[12];
        T_World_Estar(0, 1) = recv_data[13];
        T_World_Estar(0, 2) = recv_data[14];
        T_World_Estar(1, 0) = recv_data[15];
        T_World_Estar(1, 1) = recv_data[16];
        T_World_Estar(1, 2) = recv_data[17];
        T_World_Estar(2, 0) = recv_data[18];
        T_World_Estar(2, 1) = recv_data[19];
        T_World_Estar(2, 2) = recv_data[20];
        // 位置
        T_World_Estar(0, 3) = recv_data[21];
        T_World_Estar(1, 3) = recv_data[22];
        T_World_Estar(2, 3) = recv_data[23];

        q_waist(0) = recv_data[24];
        q_waist(1) = recv_data[25];
        q_waist(2) = recv_data[26];

        q_left(0) = recv_data[27];
        q_left(1) = recv_data[28];
        q_left(2) = recv_data[29];
        q_left(3) = recv_data[30];
        q_left(4) = recv_data[31];
        q_left(5) = recv_data[32];
        q_left(6) = recv_data[33];

        q_right(0) = recv_data[34];
        q_right(1) = recv_data[35];
        q_right(2) = recv_data[36];
        q_right(3) = recv_data[37];
        q_right(4) = recv_data[38];
        q_right(5) = recv_data[39];
        q_right(6) = recv_data[40];

        std::cout << "Received from client: ";
        cout << "T_World_Carbase" << T_World_Carbase << endl;
        // cout << "T_World_Estar" << T_World_Estar << endl;
        // cout << "q_waist" << q_waist << endl;
        // cout << "q_left" << q_left << endl;
        // cout << "q_right" << q_right << endl;

        car_arm.Update_parameter_single(T_World_Carbase, T_World_Estar, q_waist, q_left, q_right);
        Eigen::Matrix<double, 9, 1> x = car_arm.Solver_single();
        // cout << "x" << x.block(0, 0, 9, 1) << endl;
        cout << "p_world_leffector" << car_arm.T_World_Leffector.block(0, 3, 3, 1) << endl;
        for (int i = 0; i < 9; i++)
        {
            if (isnan(x(i)))
                x(i) = 0;
        }
        send_data[0] = x(0);
        send_data[1] = x(1);
        //
        send_data[2] = 0;
        send_data[3] = 0;
        send_data[4] = 0;
        //
        send_data[5] = x(2);
        send_data[6] = x(3);
        send_data[7] = x(4);
        send_data[8] = x(5);
        send_data[9] = x(6);
        send_data[10] = x(7);
        send_data[11] = x(8);
        //
        send_data[12] = 0;
        send_data[13] = 0;
        send_data[14] = 0;
        send_data[15] = 0;
        send_data[16] = 0;
        send_data[17] = 0;
        send_data[18] = 0;
        //
        send(new_socket, send_data, sizeof(send_data), 0);
    }

    close(new_socket);
    close(server_fd);
    return 0;
}
*/
//////////////////////////////////////////////////////double arm without velocity 没有速度 /////////////////////////////////////////////////////////////////////////////
/*
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

#define PORT 12345

int main()
{
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    double recv_data[9 + 3 + 9 + 3 + 9 + 3 + 3 + 7 + 7];
    double send_data[19];

    server_fd = socket(AF_INET, SOCK_STREAM, 0);

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 1);
    std::cout << "Server listening on port " << PORT << "...\n";

    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);

    // 开始设计
    Car_arm car_arm;
    Eigen::Matrix<double, 4, 4> T_World_Eleftstar, T_World_Erightstar, T_World_Carbase;
    T_World_Eleftstar = Eigen::Matrix<double, 4, 4>::Identity();
    T_World_Erightstar = Eigen::Matrix<double, 4, 4>::Identity();
    T_World_Carbase = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 3, 1> q_waist;
    Eigen::Matrix<double, 7, 1> q_left, q_right;

    while (true)
    {
        int r = recv(new_socket, recv_data, sizeof(recv_data), 0);

        if (r <= 0)
            break;

        T_World_Carbase(0, 0) = recv_data[0];
        T_World_Carbase(0, 1) = recv_data[1];
        T_World_Carbase(0, 2) = recv_data[2];
        T_World_Carbase(1, 0) = recv_data[3];
        T_World_Carbase(1, 1) = recv_data[4];
        T_World_Carbase(1, 2) = recv_data[5];
        T_World_Carbase(2, 0) = recv_data[6];
        T_World_Carbase(2, 1) = recv_data[7];
        T_World_Carbase(2, 2) = recv_data[8];

        // 位置
        T_World_Carbase(0, 3) = recv_data[9];
        T_World_Carbase(1, 3) = recv_data[10];
        T_World_Carbase(2, 3) = recv_data[11];

        //
        T_World_Eleftstar(0, 0) = recv_data[12];
        T_World_Eleftstar(0, 1) = recv_data[13];
        T_World_Eleftstar(0, 2) = recv_data[14];
        T_World_Eleftstar(1, 0) = recv_data[15];
        T_World_Eleftstar(1, 1) = recv_data[16];
        T_World_Eleftstar(1, 2) = recv_data[17];
        T_World_Eleftstar(2, 0) = recv_data[18];
        T_World_Eleftstar(2, 1) = recv_data[19];
        T_World_Eleftstar(2, 2) = recv_data[20];
        // 位置
        T_World_Eleftstar(0, 3) = recv_data[21];
        T_World_Eleftstar(1, 3) = recv_data[22];
        T_World_Eleftstar(2, 3) = recv_data[23];

        /////
        T_World_Erightstar(0, 0) = recv_data[24];
        T_World_Erightstar(0, 1) = recv_data[25];
        T_World_Erightstar(0, 2) = recv_data[26];
        T_World_Erightstar(1, 0) = recv_data[27];
        T_World_Erightstar(1, 1) = recv_data[28];
        T_World_Erightstar(1, 2) = recv_data[29];
        T_World_Erightstar(2, 0) = recv_data[30];
        T_World_Erightstar(2, 1) = recv_data[31];
        T_World_Erightstar(2, 2) = recv_data[32];
        // 位置
        T_World_Erightstar(0, 3) = recv_data[33];
        T_World_Erightstar(1, 3) = recv_data[34];
        T_World_Erightstar(2, 3) = recv_data[35];
        //
        q_waist(0) = recv_data[36];
        q_waist(1) = recv_data[37];
        q_waist(2) = recv_data[38];

        q_left(0) = recv_data[39];
        q_left(1) = recv_data[40];
        q_left(2) = recv_data[41];
        q_left(3) = recv_data[42];
        q_left(4) = recv_data[43];
        q_left(5) = recv_data[44];
        q_left(6) = recv_data[45];

        q_right(0) = recv_data[46];
        q_right(1) = recv_data[47];
        q_right(2) = recv_data[48];
        q_right(3) = recv_data[49];
        q_right(4) = recv_data[50];
        q_right(5) = recv_data[51];
        q_right(6) = recv_data[52];

        // std::cout << "Received from client: ";
        // cout << "T_World_Carbase" << T_World_Carbase << endl;
        // cout << "T_World_Estar" << T_World_Estar << endl;
        // cout << "q_waist" << q_waist << endl;
        // cout << "q_left" << q_left << endl;
        // cout << "q_right" << q_right << endl;

        car_arm.Update_parameter_double(T_World_Carbase, T_World_Eleftstar, T_World_Erightstar, q_waist, q_left, q_right);
        Eigen::Matrix<double, 19, 1> x = car_arm.Solver_double();
        // cout << "x" << x.block(0, 0, 9, 1) << endl;
        cout << "p_world_Leffector" << car_arm.T_World_Leffector.block(0, 3, 3, 1) << endl;
        cout << "p_world_Reffector" << car_arm.T_World_Reffector.block(0, 3, 3, 1) << endl;
        for (int i = 0; i < 19; i++)
        {
            if (isnan(x(i)))
                x(i) = 0;
        }

        send_data[0] = x(0);
        send_data[1] = x(1);
        //
        send_data[2] = x(2);
        send_data[3] = x(3);
        send_data[4] = x(4);
        //
        send_data[5] = x(5);
        send_data[6] = x(6);
        send_data[7] = x(7);
        send_data[8] = x(8);
        send_data[9] = x(9);
        send_data[10] = x(10);
        send_data[11] = x(11);
        //
        send_data[12] = x(12);
        send_data[13] = x(13);
        send_data[14] = x(14);
        send_data[15] = x(15);
        send_data[16] = x(16);
        send_data[17] = x(17);
        send_data[18] = x(18);
        //
        send(new_socket, send_data, sizeof(send_data), 0);
    }

    close(new_socket);
    close(server_fd);
    return 0;
}

*/

//
//
//
//
//
////////////////////////////////////////////////double arm with velocity ///////////////////////////////////////////////////////////////
// 用来 走轨迹  ，socket 通信 带有速度

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

#define PORT 12345

int main()
{
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    double recv_data[9 + 3 + 9 + 3 + 9 + 3 + 3 + 7 + 7 + 6 + 6]; // 。。。。
    double send_data[19];

    server_fd = socket(AF_INET, SOCK_STREAM, 0);

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 1);
    std::cout << "Server listening on port " << PORT << "...\n";

    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);

    // 开始设计
    Car_arm car_arm;
    Eigen::Matrix<double, 4, 4> T_World_Eleftstar, T_World_Erightstar, T_World_Carbase;
    T_World_Eleftstar = Eigen::Matrix<double, 4, 4>::Identity();
    T_World_Erightstar = Eigen::Matrix<double, 4, 4>::Identity();
    T_World_Carbase = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 3, 1> q_waist;
    Eigen::Matrix<double, 7, 1> q_left, q_right;

    Eigen::Matrix<double, 6, 1> v_World_left, v_World_right;

    while (true)
    {
        int r = recv(new_socket, recv_data, sizeof(recv_data), 0);

        if (r <= 0)
            break;

        T_World_Carbase(0, 0) = recv_data[0];
        T_World_Carbase(0, 1) = recv_data[1];
        T_World_Carbase(0, 2) = recv_data[2];
        T_World_Carbase(1, 0) = recv_data[3];
        T_World_Carbase(1, 1) = recv_data[4];
        T_World_Carbase(1, 2) = recv_data[5];
        T_World_Carbase(2, 0) = recv_data[6];
        T_World_Carbase(2, 1) = recv_data[7];
        T_World_Carbase(2, 2) = recv_data[8];

        // 位置
        T_World_Carbase(0, 3) = recv_data[9];
        T_World_Carbase(1, 3) = recv_data[10];
        T_World_Carbase(2, 3) = recv_data[11];

        //
        T_World_Eleftstar(0, 0) = recv_data[12];
        T_World_Eleftstar(0, 1) = recv_data[13];
        T_World_Eleftstar(0, 2) = recv_data[14];
        T_World_Eleftstar(1, 0) = recv_data[15];
        T_World_Eleftstar(1, 1) = recv_data[16];
        T_World_Eleftstar(1, 2) = recv_data[17];
        T_World_Eleftstar(2, 0) = recv_data[18];
        T_World_Eleftstar(2, 1) = recv_data[19];
        T_World_Eleftstar(2, 2) = recv_data[20];

        // 位置
        T_World_Eleftstar(0, 3) = recv_data[21];
        T_World_Eleftstar(1, 3) = recv_data[22];
        T_World_Eleftstar(2, 3) = recv_data[23];

        /////
        T_World_Erightstar(0, 0) = recv_data[24];
        T_World_Erightstar(0, 1) = recv_data[25];
        T_World_Erightstar(0, 2) = recv_data[26];
        T_World_Erightstar(1, 0) = recv_data[27];
        T_World_Erightstar(1, 1) = recv_data[28];
        T_World_Erightstar(1, 2) = recv_data[29];
        T_World_Erightstar(2, 0) = recv_data[30];
        T_World_Erightstar(2, 1) = recv_data[31];
        T_World_Erightstar(2, 2) = recv_data[32];
        // 位置
        T_World_Erightstar(0, 3) = recv_data[33];
        T_World_Erightstar(1, 3) = recv_data[34];
        T_World_Erightstar(2, 3) = recv_data[35];
        //
        q_waist(0) = recv_data[36];
        q_waist(1) = recv_data[37];
        q_waist(2) = recv_data[38];

        q_left(0) = recv_data[39];
        q_left(1) = recv_data[40];
        q_left(2) = recv_data[41];
        q_left(3) = recv_data[42];
        q_left(4) = recv_data[43];
        q_left(5) = recv_data[44];
        q_left(6) = recv_data[45];

        q_right(0) = recv_data[46];
        q_right(1) = recv_data[47];
        q_right(2) = recv_data[48];
        q_right(3) = recv_data[49];
        q_right(4) = recv_data[50];
        q_right(5) = recv_data[51];
        q_right(6) = recv_data[52];

        v_World_left(0) = recv_data[53];
        v_World_left(1) = recv_data[54];
        v_World_left(2) = recv_data[55];
        v_World_left(3) = recv_data[56];
        v_World_left(4) = recv_data[57];
        v_World_left(5) = recv_data[58];

        v_World_right(0) = recv_data[59];
        v_World_right(1) = recv_data[60];
        v_World_right(2) = recv_data[61];
        v_World_right(3) = recv_data[62];
        v_World_right(4) = recv_data[63];
        v_World_right(5) = recv_data[64];

        car_arm.Update_parameter_double_velocity(T_World_Carbase, T_World_Eleftstar, T_World_Erightstar,
                                                 v_World_left, v_World_right, q_waist, q_left, q_right);

        Eigen::Matrix<double, 19, 1> x = car_arm.Solver_double_velocity();

        // cout << "x" << x.block(0, 0, 9, 1) << endl;
        // cout << "p_world_Leffector" << car_arm.T_World_Leffector.block(0, 3, 3, 1) << endl;
        // cout << "p_world_Reffector" << car_arm.T_World_Reffector.block(0, 3, 3, 1) << endl;

        for (int i = 0; i < 19; i++)
        {
            if (isnan(x(i)))
                x(i) = 0;
        }

        send_data[0] = x(0);
        send_data[1] = x(1);
        //
        send_data[2] = x(2);
        send_data[3] = x(3);
        send_data[4] = x(4);
        //
        send_data[5] = x(5);
        send_data[6] = x(6);
        send_data[7] = x(7);
        send_data[8] = x(8);
        send_data[9] = x(9);
        send_data[10] = x(10);
        send_data[11] = x(11);
        //
        send_data[12] = x(12);
        send_data[13] = x(13);
        send_data[14] = x(14);
        send_data[15] = x(15);
        send_data[16] = x(16);
        send_data[17] = x(17);
        send_data[18] = x(18);
        //
        send(new_socket, send_data, sizeof(send_data), 0);
    }

    close(new_socket);
    close(server_fd);
    return 0;
}

///////////////////////////////////////////////////////////////test about avoid obstacle  ///////////////////////////////////////////
/*
#include "openGJK.h"

int main()
{
    // 创建并初始化多面体 A 和 B
    gkPolytope A, B;
    gkSimplex S;
    // 初始化多面体 A
    A.numpoints = 4;
    A.coord = new gkFloat *[4];
    A.coord[0] = new gkFloat[3]{0.0f, 0.0f, 0.0f};
    A.coord[1] = new gkFloat[3]{1.0f, 0.0f, 0.0f};
    A.coord[2] = new gkFloat[3]{0.0f, 1.0f, 0.0f};
    A.coord[3] = new gkFloat[3]{0.0f, 0.0f, 1.0f};

    // 初始化多面体 B
    B.numpoints = 4;
    B.coord = new gkFloat *[4];
    B.coord[0] = new gkFloat[3]{5.0f, 0.0f, 0.0f};
    B.coord[1] = new gkFloat[3]{6.0f, 0.0f, 0.0f};
    B.coord[2] = new gkFloat[3]{5.0f, 1.0f, 5.0f};
    B.coord[3] = new gkFloat[3]{5.0f, 0.0f, 1.0f};

    // 初始化单纯形 S

    // auto start = std::chrono::high_resolution_clock::now();
    gkFloat dist = compute_minimum_distance(A, B, &S);
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> elapsed = end - start;
    // std::cout << "Elapsed time: " << elapsed.count() << " ms\n";

    std::cout << "Minimum distance between A and B: " << dist << std::endl;
    return 0;
}
*/

///////////////////////////////////////////////////////////////double arm velocity collision ////////////////////////////////////////
/*
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

#define PORT 12345

int main()
{
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    double recv_data[9 + 3 + 9 + 3 + 9 + 3 + 3 + 7 + 7 + 6 + 6]; // 。。。。
    double send_data[19];

    server_fd = socket(AF_INET, SOCK_STREAM, 0);

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 1);
    std::cout << "Server listening on port " << PORT << "...\n";

    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);

    // 开始设计
    Car_arm car_arm;
    Eigen::Matrix<double, 4, 4> T_World_Eleftstar, T_World_Erightstar, T_World_Carbase;
    T_World_Eleftstar = Eigen::Matrix<double, 4, 4>::Identity();
    T_World_Erightstar = Eigen::Matrix<double, 4, 4>::Identity();
    T_World_Carbase = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 3, 1> q_waist;
    Eigen::Matrix<double, 7, 1> q_left, q_right;

    Eigen::Matrix<double, 6, 1> v_World_left, v_World_right;

    while (true)
    {
        int r = recv(new_socket, recv_data, sizeof(recv_data), 0);

        if (r <= 0)
            break;

        T_World_Carbase(0, 0) = recv_data[0];
        T_World_Carbase(0, 1) = recv_data[1];
        T_World_Carbase(0, 2) = recv_data[2];
        T_World_Carbase(1, 0) = recv_data[3];
        T_World_Carbase(1, 1) = recv_data[4];
        T_World_Carbase(1, 2) = recv_data[5];
        T_World_Carbase(2, 0) = recv_data[6];
        T_World_Carbase(2, 1) = recv_data[7];
        T_World_Carbase(2, 2) = recv_data[8];

        // 位置
        T_World_Carbase(0, 3) = recv_data[9];
        T_World_Carbase(1, 3) = recv_data[10];
        T_World_Carbase(2, 3) = recv_data[11];

        //
        T_World_Eleftstar(0, 0) = recv_data[12];
        T_World_Eleftstar(0, 1) = recv_data[13];
        T_World_Eleftstar(0, 2) = recv_data[14];
        T_World_Eleftstar(1, 0) = recv_data[15];
        T_World_Eleftstar(1, 1) = recv_data[16];
        T_World_Eleftstar(1, 2) = recv_data[17];
        T_World_Eleftstar(2, 0) = recv_data[18];
        T_World_Eleftstar(2, 1) = recv_data[19];
        T_World_Eleftstar(2, 2) = recv_data[20];

        // 位置
        T_World_Eleftstar(0, 3) = recv_data[21];
        T_World_Eleftstar(1, 3) = recv_data[22];
        T_World_Eleftstar(2, 3) = recv_data[23];

        /////
        T_World_Erightstar(0, 0) = recv_data[24];
        T_World_Erightstar(0, 1) = recv_data[25];
        T_World_Erightstar(0, 2) = recv_data[26];
        T_World_Erightstar(1, 0) = recv_data[27];
        T_World_Erightstar(1, 1) = recv_data[28];
        T_World_Erightstar(1, 2) = recv_data[29];
        T_World_Erightstar(2, 0) = recv_data[30];
        T_World_Erightstar(2, 1) = recv_data[31];
        T_World_Erightstar(2, 2) = recv_data[32];
        // 位置
        T_World_Erightstar(0, 3) = recv_data[33];
        T_World_Erightstar(1, 3) = recv_data[34];
        T_World_Erightstar(2, 3) = recv_data[35];
        //
        q_waist(0) = recv_data[36];
        q_waist(1) = recv_data[37];
        q_waist(2) = recv_data[38];

        q_left(0) = recv_data[39];
        q_left(1) = recv_data[40];
        q_left(2) = recv_data[41];
        q_left(3) = recv_data[42];
        q_left(4) = recv_data[43];
        q_left(5) = recv_data[44];
        q_left(6) = recv_data[45];

        q_right(0) = recv_data[46];
        q_right(1) = recv_data[47];
        q_right(2) = recv_data[48];
        q_right(3) = recv_data[49];
        q_right(4) = recv_data[50];
        q_right(5) = recv_data[51];
        q_right(6) = recv_data[52];

        v_World_left(0) = recv_data[53];
        v_World_left(1) = recv_data[54];
        v_World_left(2) = recv_data[55];
        v_World_left(3) = recv_data[56];
        v_World_left(4) = recv_data[57];
        v_World_left(5) = recv_data[58];

        v_World_right(0) = recv_data[59];
        v_World_right(1) = recv_data[60];
        v_World_right(2) = recv_data[61];
        v_World_right(3) = recv_data[62];
        v_World_right(4) = recv_data[63];
        v_World_right(5) = recv_data[64];

        auto start = std::chrono::high_resolution_clock::now();

        car_arm.Update_parameter(T_World_Carbase, T_World_Eleftstar, T_World_Erightstar,
                                 v_World_left, v_World_right, q_waist, q_left, q_right);

        Eigen::Matrix<double, 19, 1> x = car_arm.Solver2();

        for (int i = 0; i < 19; i++)
        {
            if (isnan(x(i)))
                x(i) = 0;
        }

        send_data[0] = x(0);
        send_data[1] = x(1);
        //
        send_data[2] = x(2);
        send_data[3] = x(3);
        send_data[4] = x(4);
        //
        send_data[5] = x(5);
        send_data[6] = x(6);
        send_data[7] = x(7);
        send_data[8] = x(8);
        send_data[9] = x(9);
        send_data[10] = x(10);
        send_data[11] = x(11);
        //
        send_data[12] = x(12);
        send_data[13] = x(13);
        send_data[14] = x(14);
        send_data[15] = x(15);
        send_data[16] = x(16);
        send_data[17] = x(17);
        send_data[18] = x(18);
        //
        send(new_socket, send_data, sizeof(send_data), 0);
    }

    close(new_socket);
    close(server_fd);

    return 0;
}
*/
// ---------------------------------tast _ other function ----------------------------------------------
/*
#include "other_function.h"

int main()
{
    double dis_left_right, dis_left_T3, dis_left_T2, dis_right_T3, dis_right_T2;
    Eigen::Matrix<double, 17, 1> q;
    q << 0, 0, 0,
        1.5708, -1.5708, -1.5708, -1.0, 0, -0.5, 0,
        -1.5708, 1.5708, 1.5708, 1, 0, 0.5, 0;
    dis_collision_self(q, dis_left_right, dis_left_T3, dis_left_T2, dis_right_T3, dis_right_T2);
    cout << dis_left_right << endl;
    return 0;
}
*/