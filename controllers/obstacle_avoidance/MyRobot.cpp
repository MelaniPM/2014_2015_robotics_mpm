/**
 * @file    MyRobot.cpp
 * @brief   cpp file to initialize the different functions and methods of the controller.
 *
 * @author  Melania Prieto
 * @date    2014-10
 */

#include "MyRobot.h"


//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    //Constants definitions
    _time_step = 64;
    _left_speed = 0;
    _right_speed = 0;

    //Initially the robot will be moving towards the other side of the world
    _mode = GOING_TO_FINAL_DESTINATION;

    //Get and enable the compass device and the distance sensors
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    for (int i = 0; i < NUM_MAX_DISTANCE_SENSOR; i++) {
        //Here concatenation functions for strings are used, to group an string with an integer.
        string str="ds";
        stringstream ss;
        ss<<i;
        str+=ss.str();
        _distance_sensor[i] = getDistanceSensor(str);
        _distance_sensor[i]-> enable(_time_step);
    }
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    //Disable compass and distance sensors
    for (int i = 0; i < NUM_MAX_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
    _my_compass->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double ir[NUM_MAX_DISTANCE_SENSOR];
    for (int i = 0; i < 4; i++) {
        ir[i] = 0;
    }
    for (int i = 15; i > 11; i--) {
        ir[i] = 0;
    }
    double compass_angle;

    while (step(_time_step) != -1) {
        // Read sensors and compass
        for (int i = 0; i < 4; i++) {
            ir[i] = _distance_sensor[i]-> getValue();
        }
        for (int i = 15; i > 11; i--) {
            ir[i] = _distance_sensor[i]-> getValue();
        }
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;
        cout << ir[0] << " " <<ir[1]<<" "<<ir[15]<<" "<<ir[14]<<" "<<ir[12]<<endl;

        // Control logic of the robot
        if (_mode == OBSTACLE_AVOID){
            if ((ir[0]!=0)||(ir[15] != 0)||(ir[1]!=0)||(ir[14] != 0)){
                //Continue avoiding collision if it detects something with the corresponding sensors
                _mode = OBSTACLE_AVOID;
                cout << "Avoiding collision." << endl;
            }
            else{
                //If nothing is detected once the obstacle is avoided move to the left
                _mode = TURN_RIGHT;
            }
        }
        else{
            if((ir[0] == 0)&&(ir[1] == 0)&&(ir[14] == 0)&&(ir[15] == 0)&&(ir[13] == 0)&&(ir[2] == 0)){
                //If it is not detecting anything and not avoiding collision,the robot will try to reach 45º
                _mode=GOING_TO_FINAL_DESTINATION;
                cout << "Trying to reach the other side." << endl;
            }
            else{
                if ((ir[0] < DISTANCE_LIMIT + 50 )&&(ir[14] < DISTANCE_LIMIT + 20)&&(ir[15] < DISTANCE_LIMIT + 50)&&(ir[1] < DISTANCE_LIMIT + 20)){
                    if ((ir[1]>ir[14])||(ir[2]>ir[13]))
                    {
                        //Moving right
                        _mode = TURN_RIGHT;
                        cout << "Turning right" << endl;
                    }
                    else{
                        //Moving left
                        _mode = TURN_LEFT;
                        cout << "Turning left" << endl;
                    }
                }
                else{
                    if ((ir[3] > DISTANCE_LIMIT )&&(ir[12]> DISTANCE_LIMIT)){
                        //Change to obstacle avoiding method
                        _mode = OBSTACLE_AVOID;
                        cout << "Avoiding collision";
                    }
                    else {
                        //Backing up
                        _mode = BACKWARDS;
                    }
                }
            }
        }

        // Send actuators commands according to the mode
        switch (_mode){
        case STOP:
            _left_speed = 0;
            _right_speed = 0;
            break;
        case FORWARD:
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED;
            break;

        case GOING_TO_FINAL_DESTINATION:
            final_destination(compass_angle,ir[12], ir[13]);
            break;
        case TURN_LEFT:
            _left_speed = MAX_SPEED/ 4.0;
            _right_speed = MAX_SPEED/ 1.25;
            break;
        case TURN_RIGHT:
            _left_speed = MAX_SPEED/1.25;
            _right_speed = MAX_SPEED /4;
            break;
        case OBSTACLE_AVOID:
            _left_speed = -MAX_SPEED / 15.0;
            _right_speed = -MAX_SPEED / 1.5;
            break;
        case BACKWARDS:
            _left_speed = -MAX_SPEED;
            _right_speed = -MAX_SPEED;
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

void MyRobot::final_destination(const double compass, const int sensor1, const int sensor2)
{
    if (compass < (DESIRED_ANGLE - 2)) {
        if (sensor1 != 0){
            // Turn left
            _left_speed = MAX_SPEED- 20;
            _right_speed = MAX_SPEED;
            cout << "LEFT" << endl;
        }
        else{
            // Turn right
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED - 20;
            cout << "RIGHT" << endl;
        }
    }
    else {
        if (compass > (DESIRED_ANGLE + 2)) {
            if (sensor2 != 0){
                // Turn right
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED - 20;
                cout << "RIGHT" << endl;
            }
            else{
                // Turn left
                _left_speed = MAX_SPEED - 20;
                _right_speed = MAX_SPEED;
                cout << "LEFT" << endl;
            }
        }
        else {
            // Move straight forward
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED;
            cout << "FORWARD" << endl;
        }
    }
}

//////////////////////////////////////////////////////
