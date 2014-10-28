/**
 * @file    MyRobot.cpp
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  Melania Prieto
 * @date    2014-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _mode = FORWARD;

    // Get and enable the compass device and the distance sensors
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _distance_sensor[0] = getDistanceSensor("ds14");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds15");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds0");
    _distance_sensor[2]->enable(_time_step);
    _distance_sensor[3] = getDistanceSensor("ds1");
    _distance_sensor[3]->enable(_time_step);
    
}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
    _my_compass->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;
    double ir14_val = 0.0, ir15_val = 0.0, ir0_val = 0.0, ir1_val = 0.0;

    while (step(_time_step) != - 1 ) {
        // Read the sensors
        const double *compass_val = _my_compass->getValues();
        ir14_val = _distance_sensor[0]->getValue();
        ir15_val = _distance_sensor[1]->getValue();
        ir0_val = _distance_sensor[2]->getValue();
        ir1_val = _distance_sensor[3]->getValue();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;   

        //Show distance sensor values
        cout << "ds14: " << ir14_val << " ds15: " << ir15_val << " ds0: " << ir0_val << " ds1: " << ir1_val << endl;

        // Robot Control logic
        if ((ir15_val >= DISTANCE_LIMIT)|| (ir0_val >= DISTANCE_LIMIT)) {
            _mode = STOP;
            cout << "Stoping" << endl;
        }
        else {
            _mode = FORWARD;
            cout << "Moving forward." << endl;
        }

        // Send actuators commands according to the mode
        switch (_mode){
            case STOP:
                    _left_speed = 0;
                    _right_speed = 0;
                    break;
            case FORWARD:
            {
                if (compass_angle < (DESIRED_ANGLE - 2)) {
                    // Turn right
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED - 15;
                }
                else {
                    if (compass_angle > (DESIRED_ANGLE + 2)) {
                        // Turn left
                        _left_speed = MAX_SPEED - 15;
                        _right_speed = MAX_SPEED;
                    }
                    else {
                        // Move straight forward
                        _left_speed = MAX_SPEED;
                        _right_speed = MAX_SPEED;
                    }
                }
                    break;
            }
            default:
                    break;
            }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
        cout <<"The robot speed is (" << _left_speed <<") ("<< _right_speed <<")\n "<< endl;
    }
}


//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////
