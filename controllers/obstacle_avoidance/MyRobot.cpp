/**
 * @file    MyRobot.cpp
 * @brief   A template for webots projects.
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

    _mode = GOING_TO_FINAL_DESTINATION;

    //Get and enable the compass device and the distance sensors
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[2]->enable(_time_step);
    _distance_sensor[3] = getDistanceSensor("ds3");
    _distance_sensor[3]->enable(_time_step);
    _distance_sensor[4] = getDistanceSensor("ds4");
    _distance_sensor[4]->enable(_time_step);
    _distance_sensor[5] = getDistanceSensor("ds5");
    _distance_sensor[5]->enable(_time_step);
    _distance_sensor[6] = getDistanceSensor("ds6");
    _distance_sensor[6]->enable(_time_step);
    _distance_sensor[7] = getDistanceSensor("ds7");
    _distance_sensor[7]->enable(_time_step);
    _distance_sensor[8] = getDistanceSensor("ds8");
    _distance_sensor[8]->enable(_time_step);
    _distance_sensor[9] = getDistanceSensor("ds9");
    _distance_sensor[9]->enable(_time_step);
    _distance_sensor[10] = getDistanceSensor("ds10");
    _distance_sensor[10]->enable(_time_step);
    _distance_sensor[11] = getDistanceSensor("ds11");
    _distance_sensor[11]->enable(_time_step);
    _distance_sensor[12] = getDistanceSensor("ds12");
    _distance_sensor[12]->enable(_time_step);
    _distance_sensor[13] = getDistanceSensor("ds13");
    _distance_sensor[13]->enable(_time_step);
    _distance_sensor[14] = getDistanceSensor("ds14");
    _distance_sensor[14]->enable(_time_step);
    _distance_sensor[15] = getDistanceSensor("ds15");
    _distance_sensor[15]->enable(_time_step);
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
    double ir0_val = 0.0, ir1_val = 0.0, ir2_val = 0.0, ir3_val = 0.0, ir12_val = 0.0, ir13_val = 0.0, ir14_val = 0.0, ir15_val = 0.0;
    double compass_angle;

    while (step(_time_step) != -1) {
        // Read the sensors
        ir0_val = _distance_sensor[0]->getValue();
        ir1_val = _distance_sensor[1]->getValue();
        ir2_val = _distance_sensor[2]->getValue();
        ir3_val = _distance_sensor[3]->getValue();


        ir12_val = _distance_sensor[12]->getValue();
        ir13_val = _distance_sensor[13]->getValue();
        ir14_val = _distance_sensor[14]->getValue();
        ir15_val = _distance_sensor[15]->getValue();
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;
        cout << ir0_val << " " <<ir1_val<<" "<<ir15_val<<" "<<ir14_val<<" "<<ir12_val<<endl;



            // Wall following
            if (_mode = GOING_TO_FINAL_DESTINATION) {

                if((ir15_val > DISTANCE_LIMIT - 50)||(ir0_val > DISTANCE_LIMIT - 50)||(ir14_val > DISTANCE_LIMIT - 50)||(ir1_val > DISTANCE_LIMIT - 50)||(ir13_val > DISTANCE_LIMIT - 50)||(ir2_val > DISTANCE_LIMIT - 50)){
                    if ((ir15_val + 50 < ir0_val)||(ir14_val < ir1_val)){
                    _mode=WALL_FOLLOWER_LEFT;
                        cout << "0." << endl;
                    }
                    else{
                        _mode = WALL_FOLLOWER_RIGHT;
                        cout << "Avoiding collision with a wall .1" << endl;
                    }
                }

            }
            else {

                    if ((ir13_val > DISTANCE_LIMIT + 20)||(ir2_val < DISTANCE_LIMIT - 20)) {
                        _mode = TURN_LEFT;
                        cout << "Turning left. 2." << endl;
                    }
                    if ((ir13_val < DISTANCE_LIMIT - 20)||(ir2_val > DISTANCE_LIMIT + 20)) {
                        _mode = TURN_RIGHT;
                        cout << "Turning right. 3." << endl;
                    }

                    else {
                        _mode = FORWARD;
                        cout<<"BÑABÑA"<<endl;
                    }
                }




        // Send actuators commands according to the mode
        switch (_mode){
        case WALL_FOLLOWER_RIGHT:
            _left_speed = - MAX_SPEED/5.0;
            _right_speed = - MAX_SPEED/20.0;
            break;
        case WALL_FOLLOWER_LEFT:
            _left_speed = - MAX_SPEED/20.0;
            _right_speed = - MAX_SPEED/5.0;
            break;
        case STOP:
            _left_speed = 0;
            _right_speed = 0;
            break;
        case FORWARD:
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED;
            break;

        case GOING_TO_FINAL_DESTINATION:
            if (compass_angle < (DESIRED_ANGLE - 2)) {
                // Turn right
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED - 20;
            }
            else {
                if (compass_angle > (DESIRED_ANGLE + 2)) {
                    // Turn left
                    _left_speed = MAX_SPEED - 20;
                    _right_speed = MAX_SPEED;
                }
                else {
                    // Move straight forward
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED;
                }
            }
            break;
        case TURN_LEFT:
            _left_speed = MAX_SPEED/ 4.0;
            _right_speed = MAX_SPEED/1.25;
            break;
        case TURN_RIGHT:
            _left_speed = MAX_SPEED/1.25;
            _right_speed = MAX_SPEED /4;
            break;
        case OBSTACLE_AVOID_LEFT:
            _left_speed = -MAX_SPEED / 4.0;
            _right_speed = -MAX_SPEED / 20.0;
            break;
        case OBSTACLE_AVOID_RIGHT:
            _left_speed = -MAX_SPEED / 20.0;
            _right_speed = -MAX_SPEED / 4.0;
            break;
        default:
            break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
        cout << _left_speed << _right_speed<< endl;
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


