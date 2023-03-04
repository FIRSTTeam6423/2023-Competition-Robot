// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * ArmUtilConstants
     */
    public static final int ARM1 = 9;
    public static final int ARM2 = 10;
    public static final int WRIST = 11;

    public static final int WRIST_LIMIT_SWITCH = 4;
    public static final int ARM_LIMIT_SWITCH = 5;

    public static final double ARM_CONVERSION_FACTOR = 1;
    public static final double ARM_LOWER_LIMIT=0;//this is not the actual value
    public static final double ARM_UPPER_LIMIT=0.5;//this is not the actual value
    public static final double WRIST_LOWER_LIMIT=0;//this is not the actual value
    public static final double WRIST_UPPER_LIMIT=0.5;//this is not the actual value

    public static final double DEADBAND = 3.5;
    public static final double TURN_P_VALUE = 0.6;
    /**
     * DriveUtil Constants
     */
    public static final int FRONTLEFT_DRIVE = 1;
    public static final int FRONTLEFT_PIVOT = 2;
    public static final int FRONTRIGHT_DRIVE = 3;
    public static final int FRONTRIGHT_PIVOT = 4;
    public static final int BACKLEFT_DRIVE = 5;
    public static final int BACKLEFT_PIVOT = 6;
    public static final int BACKRIGHT_DRIVE = 7;
    public static final int BACKRIGHT_PIVOT = 8;
    public static final int ARM_MOTOR = 9;
    public static final int WRIST_MOTOR = 10;

    public static final int TOPLEFT_ABS_ENCODER = 0;
    public static final int TOPRIGHT_ABS_ENCODER = 1;
    public static final int BOTTOMLEFT_ABS_ENCODER = 2;
    public static final int BOTTOMRIGHT_ABS_ENCODER = 3;

    public static final double TOPLEFT_ABS_ENCODER_OFFSET = 12.0;//77.1;
    public static final double TOPRIGHT_ABS_ENCODER_OFFSET = 171.6;//-82.4;
    public static final double BOTTOMLEFT_ABS_ENCODER_OFFSET = 30.8;//-115.2;
    public static final double BOTTOMRIGHT_ABS_ENCODER_OFFSET = 83.8;//4.9;

    public static final double[] ABS_ENCODER_OFFSETS = {
        TOPLEFT_ABS_ENCODER_OFFSET,
        TOPRIGHT_ABS_ENCODER_OFFSET,
        BOTTOMLEFT_ABS_ENCODER_OFFSET,
        BOTTOMRIGHT_ABS_ENCODER_OFFSET
    };

    public static final double WHEEL_RADIUS = 0.5;
    public static final double MAX_ANGULAR_SPEED = 40*Math.PI; //1/2 rotation per second
    public static final double MAX_LINEAR_SPEED = 100; //meters per second

    public static final double MODULEDRIVE_P = 0.01;//0.01;
    public static final double MODULEDRIVE_I = 0;
    public static final double MODULEDRIVE_D = 0;
    public static final double MODULEPIVOT_P = 0.005;//0.01;
    public static final double MODULEPIVOT_I = 0;
    public static final double MODULEPIVOT_D = 0;

    public static final double XDIR_P = 0;
    public static final double YDIR_P = 0;
    public static final double ROT_P = 0;

    //Pivot
    public static final double COUNTS_PER_REV = 4096; //May need to be 1024 for scaling. test to see
    public static final double RATIOED_COUNTS_PER_REV = COUNTS_PER_REV/13.71; //Takes gear reduction into account
    //Drive
    public static final double METERS_PER_REV = 2* 0.102 * Math.PI;
    public static final double TICKS_PER_METER = 4096 / METERS_PER_REV;
            
    //Ticks per rotation/360

    /**
     * We define 0 degrees as what North
     * would be, and it goes clockwise
     * 
     * x-values can be wheel base /2
     * y-values can be track/2
     * keep signs
     */
    public static final double TOPLEFT_X = 0.224;
    public static final double TOPLEFT_Y = 0.224;
    public static final double TOPLEFT_ANGLE = 45;
    public static final double TOPRIGHT_X = 0.224;
    public static final double TOPRIGHT_Y = -0.224;
    public static final double TOPRIGHT_ANGLE = 315;
    public static final double BOTTOMLEFT_X = -0.224;
    public static final double BOTTOMLEFT_Y = 0.224;
    public static final double BOTTOMLEFT_ANGLE = 135;
    public static final double BOTTOMRIGHT_X = -0.224;
    public static final double BOTTOMRIGHT_Y = -0.224;
    public static final double BOTTOMRIGHT_ANGLE = 225;
    /**
     * Controller Input Device Mapping
     * 
     */
    public static final int XBOX_DRIVER = 0;
    public static final int XBOX_OPERATOR = 1;

    public static final double XBOX_STICK_DEADZONE_WIDTH=0.1;
}