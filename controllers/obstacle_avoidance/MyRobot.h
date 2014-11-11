/**
 * @file    obstacle_avoidance.cpp
 * @brief   Controller used for reaching a final destination, without hiting obstacles.
 *
 * @author  Melania Prieto
 * @date    2014-10
 */

//The different libraries needed. Some extra libraries are needed for concatenation of strings
#include <string>
#include <iostream>
#include <sstream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_MAX_DISTANCE_SENSOR 16
#define DISTANCE_LIMIT      100
#define MAX_SPEED           50
#define DESIRED_ANGLE       45.0

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;

        Compass * _my_compass;
        DistanceSensor * _distance_sensor[NUM_MAX_DISTANCE_SENSOR];
        double _left_speed, _right_speed;

        enum Mode {
            STOP,
            WALL_FOLLOWER_LEFT,
            WALL_FOLLOWER_RIGHT,
            FORWARD,
            GOING_TO_FINAL_DESTINATION,
            TURN_LEFT,
            TURN_RIGHT,
            OBSTACLE_AVOID,
            BACKWARDS
        };

        Mode _mode;

        /**
          * @brief A function that controls the logic for the robot while there are no obstacles to avoid.
          * @param Compass states the values the compass read, while sensor1 and 2 are some specific distance sensors values.
          * @return It returns nothing. It's a control logic loop.
          */
        void final_destination(const double compass,const int sensor1, const int sensor2);

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
