/**
 * @file    MyRobot.cpp
 * @brief   .cpp file to initialize and implement the different methods and functions.
 *
 * @author  Melania Prieto
 * @date    2014-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    //Constants definitions
    _time_step = 64;
    _left_speed = 0;
    _right_speed = 0;

    //Initially the robot will be moving forward
    _mode = FORWARD;

    //Get and enable the compass device and the encoders
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
    enableEncoders(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    //Disable the compass device and the encoders
    _my_compass->disable();
    disableEncoders();
}

//////////////////////////////////////////////

void MyRobot::run()
{

    //Declaration of needed variables
    double compass_angle, final_angle, position_x,position_z, increment_x = 0,increment_z = 0, increment_angle = 0, sine, cosine, _right_speed_radians,_left_speed_radians;
    double _encoder_right,_encoder_left;

    while (step(_time_step) != -1) {
        // Read the compass:
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        //Obtain values from each enocoder
        _encoder_right = getRightEncoder();
        _encoder_left = getLeftEncoder();

        // Process sensor data here (logic control)
        final_angle = compass_angle*PI/180.0;
        _right_speed_radians=_right_speed/10;
        _left_speed_radians=_left_speed/10;

        //Definition of some needed variables
        increment_angle = increment_angle - final_angle;
        sine=sin(increment_angle);
        increment_x=RADIUS*_right_speed_radians*_time_step*sine/2000 + RADIUS*_left_speed_radians*_time_step*sine/2000;
        increment_z=RADIUS*_encoder_right/(5*2) + RADIUS*_encoder_left/(5*2);
        //In this case, as we are using the encoders to calculate the position in z, the cosine wont be needed
        // cosine=cos(increment_angle);

        //Use of odometry principles to calculate the robot position
        position_x=position_x + increment_x;
        position_z=INITIALZ + increment_z;
        cout<<"La posición actual es:"<<"\nx:"<<position_x <<"\nz:"<<position_z<<endl;

        // If the robot reaches the destination it has to stop
        if(position_z > 8){
            _mode = STOP;
        }

        // Send actuators commands according to the mode
        switch (_mode){
        case STOP:
            _left_speed = 0;
            _right_speed = 0;
            break;
        case FORWARD:
            moving_forward(compass_angle);
        default:
            break;
        }
        // Setting speed to motors:

        setSpeed(_left_speed, _right_speed);
    }
}
//////////////////////////////////////////////

void MyRobot::moving_forward(const double compass)
{
    if (compass < (DESIRED_ANGLE - 2)) {
        // Turn right
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED - 15;
    }
    else {
        if (compass > (DESIRED_ANGLE + 2)) {
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

}
//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////

