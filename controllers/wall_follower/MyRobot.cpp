/**
 * @file    main_template.cpp
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

    _mode =GOING_TO_INITIAL_POINT ;

    // Get and enable the compass device and the distance sensors
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds2");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds15");
    _distance_sensor[2]->enable(_time_step);
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
    double ir0_val = 0.0, ir2_val = 0.0, ir15_val = 0.0, compass_angle;

    while (step(_time_step) != -1) {

        // Read sensors and compass
        ir0_val = _distance_sensor[0]->getValue();
        ir2_val = _distance_sensor[1]->getValue();
        ir15_val = _distance_sensor[2]->getValue();
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);


        //Show distance sensor values
        cout << "ds15: " << ir15_val << " ds0: " << ir0_val << " ds2: " << ir2_val << endl;

        // Robot control logic
        if (_mode == GOING_TO_INITIAL_POINT) {
            // Shearch for the desired angle

            // When sufficiently close to a wall in front of robot,
            // switch mode to wall following
            if ((ir0_val > DISTANCE_LIMIT) || (ir15_val > DISTANCE_LIMIT)) {
                _mode = WALL_FOLLOWER;
                cout << "The desired initial position has been reached." << endl;
                cout << "Mode " << WALL_FOLLOWER << ": Wall following mode activated" << endl;
            }
        }
        else {

            // Wall following
            if ((ir0_val > DISTANCE_LIMIT) || (ir15_val > DISTANCE_LIMIT)) {
                _mode = WALL_FOLLOWER;
                cout << "Avoiding collision with a wall" << endl;
            }
            else {
                if (ir2_val > DISTANCE_LIMIT - 20) {
                    _mode = TURN_RIGHT;
                    cout << "Turning left." << endl;
                }
                else {
                    if (ir2_val < DISTANCE_LIMIT + 0) {
                        _mode = TURN_LEFT;
                        cout << "Turning right." << endl;
                    }
                    else {
                        _mode = FORWARD;
                        cout << "Moving forward." << endl;
                    }
                }
            }
        }

        // Send actuators commands according to the mode
        switch (_mode){
            case FORWARD:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
                break;
            case GOING_TO_INITIAL_POINT:
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
            case TURN_LEFT:
                _left_speed = MAX_SPEED / 1.25;
                _right_speed = MAX_SPEED;
                break;
            case TURN_RIGHT:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED / 1.25;
                break;
            case WALL_FOLLOWER:
                _left_speed = -MAX_SPEED / 20.0;
                _right_speed = -MAX_SPEED / 3.0;
                break;
            default:
                break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
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
