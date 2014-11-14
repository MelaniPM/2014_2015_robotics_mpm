#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   Declaration of the classes, parameters and methods needed for the controller.
 *
 * @author  Melania Prieto
 * @date    2014-11
 */


#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED           80
#define RADIUS          0.0825
#define DESIRED_ANGLE       45.0
#define DESIRED_ANGLE_LEFT  -45.0
#define INITIALX            0
#define INITIALZ            -9
#define HALFWAYZ            -3.7


class MyRobot : public DifferentialWheels {
    private:
        int _time_step;

        Compass * _my_compass;
        double _left_speed, _right_speed;

        enum Mode {
            STOP,
            FORWARD,
            FORWARD_LEFT,
            TURN_LEFT,
            TURN_RIGHT
        };

        Mode _mode;

        /**
          * @brief A function that controls the logic for the robot while there are no obstacles to avoid.
          * @param This function only depends on one variable.Compass states the values the compass read.
          * @return It returns nothing. It's a control logic loop.
          */
        void moving_forward(const double compass);

        /**
          * @brief A function that controls the logic for the robot while it is following the wall of the obstacle.
          * @param This function only depends on one variable.Compass states the values the compass read.
          * @return It returns nothing. It's a control logic loop.
          */
        void moving_forward_left(const double compass);

        /**
          * @brief A function that calculates the position of the robot using encoders
          * @param This function depends on the values of each encoder and an initial position
          * @return It returns the position
          */
        double calculate_position(double _encoder1, double _encoder2,double INITIAL_POSITION);

    public:
        /**
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
