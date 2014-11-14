#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.cpp
 * @brief   Controller for detecting walls using the front camera.
 *
 * @author  Melania Prieto
 * @date    2014-11
 */

#include <iostream>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED           100

class MyRobot : public DifferentialWheels {

private:
    int _time_step;

    // velocities
    double _left_speed, _right_speed;

    // sensors
    Camera *_forward_camera;
    Camera *_spherical_camera;

public:

        /**
         * @brief Empty constructor of the class.
         */
        MyRobot();

        /**
         * @brief Destructor of the class.
         */
        ~MyRobot();

        /**
         * @brief Function with the logic of the controller.
         * @param
         * @return
         */
        void run();
        
};

#endif

