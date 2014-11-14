/**
 * @file    MyRobot.cpp
 * @brief   Controller used to detect a yellow line near te robot.
 *
 * @author  Melania Prieto
 * @date    2014-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    //Default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    //Get and enable the different cameras available for the robot
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    //Disable cameras
    _forward_camera->disable();
    _spherical_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    //Variable which increases whenever the desired color is detected by the camera
    int sum = 0;
    //Variables which allow the comparison with the RGB code
    unsigned char green = 0, red = 0, blue = 0;
    //Variable that stores the porcentage of desired color present in the surroundings of the robot
    double percentage_yellow = 0.0;

    // Get size of images for forward camera
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();
    cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;

    // Get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;

    while (step(_time_step) != -1) {
        sum = 0;

        // Get current image from spherical camera
        const unsigned char *image_s = _spherical_camera->getImage();

        // Count number of pixels that are yellow
        // It is assumed to have pixel value > 240 out of 255 for green and red colors, but smaller than 2 for blue
        for (int x = 0; x < image_width_s; x++) {
            for (int y = 0; y < image_height_s; y++) {
                green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
                red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
                blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

                //When a white pixel is obtained increase by one the variable sum
                if ((green > 240) && (red > 240) && (blue < 2 )) {
                    sum = sum + 1;
                }
            }
        }

        //Calculation of the percentage
        percentage_yellow = (sum / (float) (image_width_s * image_height_s)) * 100;
        cout << "Percentage of yellow in forward camera image: " << percentage_yellow << endl;

        //Condition that satisfies the presence of a yellow line
        if(percentage_yellow > 0.25){
            cout<<"There is a yellow line near the robot position"<<endl;
        }

        // turn around slowly
        _left_speed = 5;
        _right_speed = -5;

        // set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////
