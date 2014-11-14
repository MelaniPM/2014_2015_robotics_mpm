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
    double compass_angle,position_x = 0,position_z;
    double _encoder_right,_encoder_left;

    while (step(_time_step) != -1) {
        // Read the compass:
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        //Obtain values from each enocoder
        _encoder_right = getRightEncoder();
        _encoder_left = getLeftEncoder();

        //Control logic of the robot
        if((_mode == FORWARD)&&(position_x < 3.5)){
            //Calculation of the actual position
            position_z = calculate_position(_encoder_right,_encoder_left,INITIALZ);
            cout<<"The actual postion is: ( 0 , "<<position_z<<" )"<<endl;
            // If the robot reaches the destination it has to stop
            if(position_z > (- 4)){
                _mode = TURN_LEFT;
            }
            else {
                _mode = FORWARD;
            }
        }
        else{
            //If the robot gets where the box is, turn left to avoid collision. Stop when the desired direction is obtained
            if (_mode == TURN_LEFT){
                if(compass_angle > DESIRED_ANGLE_LEFT){
                    _mode = TURN_LEFT;
                }
                else {
                    _mode = STOP;
                    setEncoders(0,0);
                }
            }
            else{
                //Start moving parallel to the box, with desired direction, previously obtained
                if ((_mode == STOP)&&(position_x < 3.5)&&(position_z < 8)){
                    _mode = FORWARD_LEFT;
                }
                //Moving parallel to the box side
                else if( _mode == FORWARD_LEFT){
                    //Calculation of the actual position
                    position_x = calculate_position(_encoder_right,_encoder_left,INITIALX);
                    cout<<"The actual postion is: ( "<<position_x<<", -3.7 )"<<endl;
                    if (position_x > 3.7){
                        _mode = TURN_RIGHT;
                    }
                    else {
                        _mode = FORWARD_LEFT;
                    }
                }
                //Control the new direction in order to obtaine the desired one
                else if (_mode == TURN_RIGHT){
                    if(compass_angle < DESIRED_ANGLE){
                        _mode = TURN_RIGHT;
                    }
                    else {
                        _mode = STOP;
                        setEncoders(0,0);
                    }
                }
                //Begin to move forward again
                else if ((_mode == STOP)&&(position_x > 3.5)&&(position_z < 8)){
                    _mode = FORWARD;
                }
                //Move forward until the desired final position is reached
                else if ((_mode == FORWARD)&&(position_x > 3.5)){
                    position_z = calculate_position(_encoder_right,_encoder_left,HALFWAYZ);
                    cout<<"The actual postion is: ( 3.92 , "<<position_z<<" )"<<endl;
                    if(position_z > 8){
                        _mode = STOP;
                    }
                    else {
                        _mode = FORWARD;
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
        case FORWARD_LEFT:
            moving_forward_left(compass_angle);
            break;
        case FORWARD:
            moving_forward(compass_angle);
            break;
        case TURN_LEFT:
            _left_speed = - MAX_SPEED/ 10.0;
            _right_speed = MAX_SPEED/ 10.0;
            break;
        case TURN_RIGHT:
            _left_speed = MAX_SPEED/10.0;
            _right_speed = - MAX_SPEED /10.0;
            break;
        default:
            break;
        }
        // Setting speed to motors:
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

double MyRobot::calculate_position(double _encoder1, double _encoder2,double INITIAL_POSITION)
{
    double increment, position;
    increment=RADIUS*_encoder1/(5*2) + RADIUS*_encoder2/(5*2);
    position=INITIAL_POSITION + increment;
    return position;
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

void MyRobot::moving_forward_left(const double compass)
{
    if (compass < (DESIRED_ANGLE_LEFT - 2)) {
        // Turn right
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED - 15;
    }
    else {
        if (compass > (DESIRED_ANGLE_LEFT + 2)) {
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
