/**
 * @file    obstacle_avoidance.cpp
 * @brief   Controller used for reaching a destination, using odometry.
 *
 * @author  Melania Prieto
 * @date    2014-11
 */

#include "MyRobot.h"

/**
 * @brief Main program.
 * @param The parameters of this function are argc y **argv
 * @return This function return just a correct functioning
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
