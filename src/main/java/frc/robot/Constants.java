// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
     * GrabUtil Constants
     */

    public static final int GRAB_MOTOR = 12; 
    public static final double GRAB_INTAKE_SPEED = 0.75;
    public static final double GRAB_OUTPUT_SPEED = -0.25;
    public static final double MIN_GRAB_INTAKE_VOLTAGE = 12.5;//probably wrong

    /**
     * ArmUtilConstants
     */
    public static final int ARM1 = 9;
    public static final int ARM2 = 10;
    public static final int WRIST = 11;

    public static final int WRIST_LIMIT_SWITCH = 8;
    public static final int ARM_LIMIT_SWITCH = 5;
    public static final int GRABBER_LIMIT_SWITCH_ID = 6;
    
    public static final double ARM_CONVERSION_FACTOR = (360.0/36.0); //fixed
    public static final double WRIST_CONVERSION_FACTOR = (360/90.0);
    public static final double ARM_LOWER_LIMIT=0;
    public static final double ARM_UPPER_LIMIT=0.5;//this is not the actual value
    public static final double WRIST_LOWER_LIMIT=0;//this is not the actual value
    public static final double WRIST_UPPER_LIMIT=0;
     
    public static final Constraints ARM_PROFILE_CONSTRAINTS = new Constraints(10, 10);
    public static final Constraints WRIST_PROFILE_CONSTRAINTS = new Constraints(500, 400);

    public static final double ARM_OSCIL_DEADBAND = 10; //degrees
    public static final double WRIST_RETRACT_DEADBAND = 15;

    public static final double DEADBAND = 3.5;
    public static final double ARM_P = 0.009;
    public static final double ARM_I = 0;
    public static final double ARM_D = 0;

    public static final double WRIST_P = 0.070;
    public static final double WRIST_I = 0;
    public static final double WRIST_D = 0;

    //ARMSTATE CONSTANTS
    //Arm has max of ~120, parrel to ground at ~80
    //Since wrist has a max of zero at limit switch
    //All degrees should be negative. otherwise, wrist will break itself
    
    public static final double HIGH_GOAL_ARM = 0;
    public static final double HIGH_GOAL_WRIST = -180;

    public static final double MIDDLE_GOAL_ARM = -12.5; // 0 is parallel to ground
    public static final double MIDDLE_GOAL_WRIST = -130;

    public static final double LOW_GOAL_ARM = -68;
    public static final double LOW_GOAL_WRIST = -80;

    public static final double HIGH_PICK_ARM = 0;
    public static final double HIGH_PICK_WRIST = -50;

    public static final double GROUND_PICK_ARM = -68;
    public static final double GROUND_PICK_WRIST = -150;

    //Feedforward constants - arbitrary values
    public static final double ARM_kG = 1.05; //0.58;//
    public static final double ARM_kG_CLAW_OUT = 1.75;
    public static final double ARM_kV = 0;//0.00765858;
    public static final double ARM_kS = 0;//0.23125;
    public static final double ARM_kA = 0;//0.00086773;
     
    public static final double WRIST_kG = 0.5;//1.3802;
    public static final double WRIST_kV = 0;//0.016472;
    public static final double WRIST_kS = 0;//-0.6777;
    public static final double WRIST_kA = 0;//0.0098395;

    public static final double ARM_VELOCITY = 1;
    public static final double ARM_ACCELERATION = 0;

    public static final double WRIST_VELOCITY = 0;
    public static final double WRIST_ACCELERATION = 0;

    public static final double WRIST_PHYSICAL_STOP = -174;
    public static final double WRIST_STOP_DEADBAND = 10;

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

    public static final double TOPLEFT_ABS_ENCODER_OFFSET = 116.233;
    public static final double TOPRIGHT_ABS_ENCODER_OFFSET = 77.01;
    public static final double BOTTOMLEFT_ABS_ENCODER_OFFSET = 73.7016 + 180;
    public static final double BOTTOMRIGHT_ABS_ENCODER_OFFSET = 131.52;

    public static final double[] ABS_ENCODER_OFFSETS = {
        TOPLEFT_ABS_ENCODER_OFFSET,
        TOPRIGHT_ABS_ENCODER_OFFSET,
        BOTTOMLEFT_ABS_ENCODER_OFFSET,
        BOTTOMRIGHT_ABS_ENCODER_OFFSET
    };

    public static final double WHEEL_RADIUS = 0.5;
    public static final double MAX_ANGULAR_SPEED = 2500; //
    public static final double MAX_LINEAR_SPEED = 29.5; //meters per second

    public static final double ALIGN_TO_TAG_MAX_VELOCITY = 2;
    public static final double ALIGN_TO_TAG_MAX_ACCELERATION = 1;

    public static final double MAX_PATH_VELOCITY = 2;
    public static final double MAX_PATH_ACCELERATION = 1;

    public static final double MODULEDRIVE_P = 0.039753;//0.0024
    public static final double MODULEDRIVE_I = 0;
    public static final double MODULEDRIVE_D = 0;
    public static final double MODULEPIVOT_P = 0.005;//0.01;
    public static final double MODULEPIVOT_I = 0;
    public static final double MODULEPIVOT_D = 0;

    public static final double AUTO_BALANCE_P = .005;
    public static final double AUTO_BALANCE_DEADBAND = 1.6;
    public static final double AUTO_BALANCE_TIME = 2;

    public static final double XDIR_P = 0;
    public static final double YDIR_P = 0;
    public static final double ROT_P = 0;

    public static final Transform3d CAMERA_TO_ROBOT=new Transform3d(new Translation3d(-.0635, .1778, 0.0), new Rotation3d()); //Dummy

    public static final double APRIL1_X = 15.51;
        public static final double APRIL1_Y = 1.07;
        public static final double APRIL1_ROT = 180;
        public static final double APRIL2_X = 15.51;
        public static final double APRIL2_Y = 2.75;
        public static final double APRIL2_ROT = 180;
        public static final double APRIL3_X = 15.51;
        public static final double APRIL3_Y = 3.74;
        public static final double APRIL3_ROT = 180;
        public static final double APRIL4_X = 16.18;
        public static final double APRIL4_Y = 4.42;
        public static final double APRIL4_ROT = 180;
        public static final double APRIL5_X = 0.36;
        public static final double APRIL5_Y = 4.42;
        public static final double APRIL5_ROT = 0;
        public static final double APRIL6_X = 1.03;
        public static final double APRIL6_Y = 4.42;
        public static final double APRIL6_ROT = 0;
        public static final double APRIL7_X = 1.03;
        public static final double APRIL7_Y = 3.74;
        public static final double APRIL7_ROT = 0;
        public static final double APRIL8_X = 1.03;
        public static final double APRIL8_Y = 1.07;
        public static final double APRIL8_ROT = 0;
        public static final double GRID_TAG_HEIGHT = .36;//METERS /0.46; // Tags 1-3 (red) & 6-8 (blue)
        public static final double SUB_TAG_HEIGHT = .59;// 0.67; //Tags 4-5
        public static final Pose3d[] TagPoses = {
            new Pose3d(APRIL1_X,APRIL1_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL1_ROT))),
            new Pose3d(APRIL2_X,APRIL2_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL2_ROT))),
            new Pose3d(APRIL3_X,APRIL3_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL3_ROT))),
            new Pose3d(APRIL4_X,APRIL4_Y,SUB_TAG_HEIGHT,  new Rotation3d(0, 0, Math.toRadians(APRIL4_ROT))),
            new Pose3d(APRIL5_X,APRIL5_Y,SUB_TAG_HEIGHT,  new Rotation3d(0, 0, Math.toRadians(APRIL5_ROT))),
            new Pose3d(APRIL6_X,APRIL6_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL6_ROT))),
            new Pose3d(APRIL7_X,APRIL7_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL7_ROT))),
            new Pose3d(APRIL8_X,APRIL8_Y,GRID_TAG_HEIGHT, new Rotation3d(0, 0, Math.toRadians(APRIL8_ROT))),
            };
            
    //Ticks per rotation/360

    /**
     * We define 0 degrees as what North
     * would be, and it goes clockwise
     * 
     * x-values can be wheel base /2
     * y-values can be track/2
     * keep signs
     */
    //NOTE:
    //X IS FORWARD/BACKWARD NOT LEFT RIGHT
    //Y IS LEFT/RIGHT
    public static final double TOPLEFT_X = 0.224;
    public static final double TOPLEFT_Y = 0.224; //swap to negative
    public static final double TOPLEFT_ANGLE = 45;
    public static final double TOPRIGHT_X = 0.224;
    public static final double TOPRIGHT_Y = -0.224; //swap to positive
    public static final double TOPRIGHT_ANGLE = 315;
    public static final double BOTTOMLEFT_X = -0.224;
    public static final double BOTTOMLEFT_Y = 0.224; //swap to negative
    public static final double BOTTOMLEFT_ANGLE = 135;
    public static final double BOTTOMRIGHT_X = -0.224;
    public static final double BOTTOMRIGHT_Y = -0.224; //swap to positve
    public static final double BOTTOMRIGHT_ANGLE = 225;
    /**
     * Controller Input Device Mapping
     * 
     */
    public static final int XBOX_DRIVER = 1;
    public static final int XBOX_OPERATOR = 2;
    public static final int JOYSTICK_OPERATOR = 0;//1;
    
    public static final double ARM_JOYSTICK_INPUT_DEADBAND = .25;

    public static final double XBOX_STICK_DEADZONE_WIDTH=0.05;

    public static final double AUTO_X_P = 9;
    public static final double AUTO_X_I = 0;
    public static final double AUTO_X_D = 0;

    public static final double AUTO_Y_P = 9;
    public static final double AUTO_Y_I = 0;
    public static final double AUTO_Y_D = 0;

    public static final double AUTO_THETA_P = .35;
    public static final double AUTO_THETA_I = .035;
    public static final double AUTO_THETA_D = 4.5;
}