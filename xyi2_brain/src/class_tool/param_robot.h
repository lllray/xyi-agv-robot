#ifndef __param_robot_H
#define __param_robot_H

    #include <iostream>
    using namespace std;


    /*
        00000  <width>      ^ x
        00m00               |
        00000               |
                   y <--- robot_r
       <height>
    */

    typedef class PARAM_ROBOT
    {
    public:
        PARAM_ROBOT()
        {
            Robot_Radius_Max= 0.0f;

            Robot_Origin[0] = 0.0f, Robot_Origin[1] = 0.0f;
            Robot_Width = 0.0f, Robot_Height = 0.0f;
        }

        ~PARAM_ROBOT()
        {

        }

        float Robot_Radius_Max;

        float Robot_Origin[2];
        float Robot_Width, Robot_Height;



    private:


    }PARAM_ROBOT;



#endif



