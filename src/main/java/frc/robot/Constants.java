/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class PID {
        public static final class Aim {
            public static final double KP = 0.05f;
            public static final double KI = 0;
            public static final double KD = 0.001;

            public static final double FEED_FORWARD = 0.35f;

            public static final double POS_TOL = 0.8f;
            public static final double VEL_TOL = 100; 
        }
    }

    public static final int LEFT_FRONT_DRIVE = 4;
    public static final int LEFT_BACK_DRIVE = 3;
    public static final int RIGHT_FRONT_DRIVE = 9;
    public static final int RIGHT_BACK_DRIVE = 8;
    public static final int PICKUP = 2;
    public static final int SHOOTER = 1;
    public static final int INDEX = 5;
    public static final int COLOR_WHEEL = 10;
    public static final int COLOR_WHEEL_ARM = 11;
    public static final int CLIMB = 7;
    public static final int BAR_MOVE = 6;
    public static final double DEFAULT_SHOOT = -0.6;

    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;

    //Driver Joystick
    public static final int ELEVATOR_ON_BUTTON = 4;
    public static final int ELEVATOR_OFF_BUTTON = 3;
    public static final int ELEVATOR_REVERSE_BUTTON = 5;
    public static final int INVERT_CONTROL_BUTTON = 2;
    public static final int OPEN_CLIMB_BUTTON = 7;
    //Shooter Joystick
    public static final int SHOOT_BUTTON = 10;
    public static final int STOP_SHOOT_BUTTON = 11;
    public static final int INDEX_ON_BUTTON = 1;
	public static final int ARM_UP_BUTTON = 3;
    public static final int ARM_DOWN_BUTTON = 2;
    public static final int DISTANCE_SHOOT = 9;
	//public static final int WHEEL_FIND_BUTTON = 4;
    public static final int WHEEL_SPIN_BUTTON = 5;
    public static final int WHEEL_STOP_BUTTON = 4;
    public static final int AIM_BUTTON = 6;
    
    // Color Sensor port
    //public static final Port COLOR_SENSOR_PORT = Port.valueOf("1131");
    
    //Encoders
    //DIO ports
    public static final int ShooterEncode_port = 0;
    public static final int COLOR_WHEEL_UP_LS = 1;
    public static final int COLOR_WHEEL_DOWN_LS = 2;
    public static final int CLIMBER_LS_DOWN = 3;
    public static final int CLIMBER_LS_UP = 4;
    public static final int Encode_Port1 = 5;
    public static final int Encode_Port2 = 6;
    public static final int Encode_Port3 = 7;
    public static final int Encode_Port4 = 8;
    public static final int Stop_Drive_LS = 9;
}
