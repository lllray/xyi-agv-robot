#ifndef __usb_com_define_H
#define __usb_com_define_H

    #define CommandLenth 40
    #define RS_DEVICE   "/dev/ttyUSB0"

    #define BAUDRATE    115200
    #define PARITY      serial_port::parity(serial_port::parity::none)   // none, odd
    #define STOPBITS    serial_port::stop_bits::one
    #define CharSize    8

	// In:	
    #define Topics_Cmd          "/RobotPort_CmdVel"             // geometry_msgs::Twist         used
    #define Topics_PosiCmd		"/RobotPort_PosiCmd"			// geometry_msgs::Point32       unused
	
	// Out:
    #define Topics_Ack          "/RobotPort_Ack"                // std_msgs::Int8               unused
    #define Topics_RTOSOdom     "/RobotPort_Odom"               // nav_msgs::Odometry           used

    // unused but remained
    #define Topics_RTOSPoint	"/RobotPort_TmpPoint"			// geometry_msgs::Point32       unused
    #define Topics_RTOSTwist	"/RobotPort_Twist"              // geometry_msgs::Twist         unused


#endif


