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

    public static final double ARM_TICKS_PER_DEG = 0.0024414063 * 4096; //fixed
    public static final double WRIST_TICKS_PER_DEG = 0.001373291 * 4096;
    public static final double ARM_LOWER_LIMIT=0;
    public static final double ARM_UPPER_LIMIT=0.5;//this is not the actual value
    public static final double WRIST_LOWER_LIMIT=0;//this is not the actual value
    public static final double WRIST_UPPER_LIMIT=0;
     


    public static final double DEADBAND = 3.5;
    public static final double ARM_P = 0.00000024412;
    public static final double ARM_I = 0;
    public static final double ARM_D = 0.00000002828;

    public static final double WRIST_P = 0.14759;
    public static final double WRIST_I = 0;
    public static final double WRIST_D = 0.049768;

    //ARMSTATE CONSTANTS
    //Arm has max of ~120, parrel to ground at ~80
    //Since wrist has a max of zero at limit switch
    //All degrees should be negative. otherwise, wrist will break itself
    
    public static final double HIGH_GOAL_ARM = 110;
    public static final double HIGH_GOAL_WRIST = -180;

    public static final double MIDDLE_GOAL_ARM = 80;
    public static final double MIDDLE_GOAL_WRIST = -130;

    public static final double LOW_GOAL_ARM = 40;
    public static final double LOW_GOAL_WRIST = -80;

    public static final double HIGH_PICK_ARM = 95;
    public static final double HIGH_PICK_WRIST = -50;

    public static final double GROUND_PICK_ARM = 15;
    public static final double GROUND_PICK_WRIST = -150;

    //Feedforward constants - arbitrary values
    public static final double ARM_kG = 0.82056;
    public static final double ARM_kV = 0.00765858;
    public static final double ARM_kS = 0.23125;
    public static final double ARM_kA = 0.00086773;
     
    public static final double WRIST_kG = 1.3802;
    public static final double WRIST_kV = 0.016472;
    public static final double WRIST_kS = -0.6777;
    public static final double WRIST_kA = 0.0098395;

    public static final double ARM_VELOCITY = 0.5;
    public static final double ARM_ACCELERATION = 0;

    public static final double WRIST_VELOCITY = 0;
    public static final double WRIST_ACCELERATION = 0;

    public static final double ARM_FEEDFORWARD_OFFSET = 90;

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

    public static final double DRIVECONVERSIONFACTOR = (1/7.13) * .096 * Math.PI;
    public static final double DEGREES_PER_ROTATION = 360;

    public static final double TOPLEFT_ABS_ENCODER_OFFSET = 35.6;
    public static final double TOPRIGHT_ABS_ENCODER_OFFSET = 158.2;
    public static final double BOTTOMLEFT_ABS_ENCODER_OFFSET = 152.5;
    public static final double BOTTOMRIGHT_ABS_ENCODER_OFFSET = 105.7;

    public static final double[] ABS_ENCODER_OFFSETS = {
        TOPLEFT_ABS_ENCODER_OFFSET,
        TOPRIGHT_ABS_ENCODER_OFFSET,
        BOTTOMLEFT_ABS_ENCODER_OFFSET,
        BOTTOMRIGHT_ABS_ENCODER_OFFSET
    };

    public static final double WHEEL_RADIUS = 0.5;
    public static final double MAX_ANGULAR_SPEED = 500; //
    public static final double MAX_LINEAR_SPEED = 15; //meters per second

    public static final double MODULEDRIVE_P = 0.039753;//0.0024
    public static final double MODULEDRIVE_I = 0;
    public static final double MODULEDRIVE_D = 0;
    public static final double MODULEPIVOT_P = 0.005;//0.01;
    public static final double MODULEPIVOT_I = 0;
    public static final double MODULEPIVOT_D = 0;

    public static final double XDIR_P = 0;
    public static final double YDIR_P = 0;
    public static final double ROT_P = 0;
            
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