# 设置最低的CMake版本要求
cmake_minimum_required(VERSION 3.5)

# 设置项目名称
project(car_arm)

# 设置C++标准（例如C++17）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 设置源文件和头文件路径
include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

# 查找当前目录下的所有源文件
file(GLOB SOURCES
    src/*.cpp
)

find_package(OsqpEigen REQUIRED)


include_directories(/usr/local/include/kdl)
# 添加可执行文件
add_executable(main ${SOURCES})

target_link_libraries(main
    OsqpEigen::OsqpEigen
    orocos-kdl
    opengjk_ce    # 对应GJK，上面lib里面的.so
)
