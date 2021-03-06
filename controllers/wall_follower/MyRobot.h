/**
 * @file    main_template.cpp
 * @brief   A template for webots projects.
 *
 * @author  Melania Prieto
 * @date    2014-10
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 4
#define DISTANCE_LIMIT      100
#define MAX_SPEED           50
#define DESIRED_ANGLE       135.0

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;

        Compass * _my_compass;
        DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];
        double _left_speed, _right_speed;

        enum Mode {
            GOING_TO_INITIAL_POINT,
            FORWARD,
            TURN_LEFT,
            TURN_RIGHT,
            WALL_FOLLOWER
        };

        Mode _mode;

    public:
        // You may need to define your private methods or variables, like
        //  Constructors, helper functions, etc.

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
          * @brief An example for converting bearing vector from compass to angle (in degrees).
          * @param Bearing vector
          * @return Degrees
          */
        double convert_bearing_to_degrees(const double* in_vector);
};
