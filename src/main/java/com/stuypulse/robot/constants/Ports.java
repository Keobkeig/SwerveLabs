/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface SwerveDrive {
        int FRONT_LEFT_TURN = 13;
        int FRONT_LEFT_DRIVE = 12;
        int FRONT_LEFT_ENCODER = 1;

        int FRONT_RIGHT_TURN = 11;
        int FRONT_RIGHT_DRIVE = 10;
        int FRONT_RIGHT_ENCODER = 2;

        int BACK_LEFT_TURN = 15;
        int BACK_LEFT_DRIVE = 14;
        int BACK_LEFT_ENCODER = 3;

        int BACK_RIGHT_TURN = 17;
        int BACK_RIGHT_DRIVE = 16;
        int BACK_RIGHT_ENCODER = 4;
    }
}
