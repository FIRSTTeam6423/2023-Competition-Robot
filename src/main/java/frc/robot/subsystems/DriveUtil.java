// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveUtil extends SubsystemBase {
  //P denotes Pivoting, D driving
  private final Translation2d m_frontLeftLoc = new Translation2d(Constants.TOPLEFT_X, Constants.TOPLEFT_Y);
  private final Translation2d m_frontRightLoc = new Translation2d(Constants.TOPRIGHT_X, Constants.TOPRIGHT_Y);
  private final Translation2d m_backLeftLoc = new Translation2d(Constants.BOTTOMLEFT_X, Constants.TOPLEFT_Y);
  private final Translation2d m_backRightLoc = new Translation2d(Constants.BOTTOMRIGHT_X, Constants.BOTTOMRIGHT_Y);

  private final SwerveModule m_frontLeft =  new SwerveModule(Constants.TOPLEFT_DRIVE, Constants.TOPLEFT_PIVOT);
  private final SwerveModule m_frontRight =  new SwerveModule(Constants.TOPRIGHT_DRIVE, Constants.TOPRIGHT_PIVOT);
  private final SwerveModule m_backLeft =  new SwerveModule(Constants.BOTTOMLEFT_DRIVE, Constants.BOTTOMLEFT_PIVOT);
  private final SwerveModule m_backRight =  new SwerveModule(Constants.BOTTOMRIGHT_DRIVE, Constants.BOTTOMRIGHT_PIVOT);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc);

  //getPosition is just placeholder for getting distance with encoders even though wpilib uses it as an example
  // this took me like 30 min ot figure out
  // convert encoders to m
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getHeading2d(), 
  new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_backLeft.getPosition(),
    m_backRight.getPosition()
  }, new Pose2d(5.0, 13.5, new Rotation2d()));

  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  public double setpoint;

  public DriveUtil() {
    setpoint = 0;

    calibrateGyro();
  }

  public void driveRobot(boolean fieldRelative){
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(RobotContainer.getDriverLeftXboxX() * Constants.MAX_LINEAR_SPEED,  
            RobotContainer.getDriverLeftXboxY() * Constants.MAX_LINEAR_SPEED, 
            RobotContainer.getDriverRightXboxX() * Constants.MAX_ANGULAR_SPEED, getHeading2d())
            : new ChassisSpeeds(RobotContainer.getDriverLeftXboxX() * Constants.MAX_LINEAR_SPEED, 
            RobotContainer.getDriverLeftXboxY() * Constants.MAX_LINEAR_SPEED,
            RobotContainer.getDriverRightXboxX() * Constants.MAX_ANGULAR_SPEED));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_LINEAR_SPEED);
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

  }

  public Rotation2d getHeading2d(){
    return Rotation2d.fromDegrees(gyro.getYaw());
}

public Pose2d getPose(){
  return m_odometry.getPoseMeters();
}

  public double getHeading(){
    return gyro.getYaw();
  }

  public void resetGyro(){
    gyro.reset();
  }

  public void calibrateGyro(){
    gyro.calibrate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var gyroAngle = getHeading2d();

    m_odometry.update(gyroAngle,
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(), m_frontRight.getPosition(),
      m_backLeft.getPosition(), m_backRight.getPosition()
    });
  }
}