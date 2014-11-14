#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   .h file defining the desired classes for the straight_odometry controller.
 *
 * @author  Melania Prieto
 * @date    2014-10
 */


#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED       80
#define RADIUS          0.0825
#define PI              3.1415
#define DESIRED_ANGLE   45.0
#define INITIALX        0
#define INITIALZ        -9


class MyRobot : public DifferentialWheels {

private:
    int _time_step;

    Compass * _my_compass;
    double _left_speed, _right_speed;

    enum Mode {
        STOP,
        FORWARD};

    Mode _mode;

    /**
      * @brief A function that controls the logic for the robot while there are no obstacles to avoid.
      * @param This function only depends on one variable.Compass states the values the compass read.
      * @return It returns nothing. It's a control logic loop.
      */
    void moving_forward(const double compass);

public:

    /*
     * @brief Empty constructor of the class.
     * @param
     * @return
     */
    MyRobot();

    /**
     * @brief Destructor of the class.
     * @param
     * @return
     */
    ~MyRobot();

    /**
     * @brief User defined function for initializing and running the template class.
     * @param
     * @return
     */
    void run();

    /**
      * @brief A function that converts a bearing vector from compass to angle (in degrees).
      * @param Bearing vector
      * @return Degrees
      */
    double convert_bearing_to_degrees(const double* in_vector);
};
#endif
