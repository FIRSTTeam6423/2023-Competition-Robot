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
     * DriveUtil Constants
     */
    public static final int TOPLEFT_DRIVE = 1;
    public static final int TOPLEFT_PIVOT = 2;
    public static final int TOPRIGHT_DRIVE = 3;
    public static final int TOPRIGHT_PIVOT = 4;
    public static final int BOTTOMLEFT_DRIVE = 5;
    public static final int BOTTOMLEFT_PIVOT = 6;
    public static final int BOTTOMRIGHT_DRIVE = 7;
    public static final int BOTTOMRIGHT_PIVOT = 8;
    public static final int ARM_MOTOR = 9;
    public static final int WRIST_MOTOR = 10;

    public static final int TOPLEFT_ABS_ENCODER = 0;
    public static final int TOPRIGHT_ABS_ENCODER = 1;
    public static final int BOTTOMRIGHT_ABS_ENCODER = 2;
    public static final int BOTTOMLEFT_ABS_ENCODER = 3;

    public static final double WHEEL_RADIUS = 0.5;
    public static final double MAX_ANGULAR_SPEED = Math.PI; //1/2 rotation per second
    public static final double MAX_LINEAR_SPEED = 5; //meters per second

    public static final double MODULEDRIVE_P = 0.5;
    public static final double MODULEDRIVE_I = 0.5;
    public static final double MODULEDRIVE_D = 0.5;
    public static final double MODULEPIVOT_P = 0.5;
    public static final double MODULEPIVOT_I = 0.5;
    public static final double MODULEPIVOT_D = 0.5;

    //Pivot
    public static final double COUNTS_PER_REV = 4096; //May need to be 1024 for scaling. test to see
    public static final double RATIOED_COUNTS_PER_REV = COUNTS_PER_REV/13.71; //Takes gear reduction into account
    //Drive
    public static final double METERS_PER_REV = 39.37 * 4 * Math.PI; //39.37 is factor for inches to meters
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
}