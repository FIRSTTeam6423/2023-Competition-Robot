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

public class DriveUtil extends SubsystemBase {
	// P denotes Pivoting, D driving
	private final Translation2d m_frontLeftLoc = new Translation2d(Constants.TOPLEFT_X, Constants.TOPLEFT_Y);
	private final Translation2d m_frontRightLoc = new Translation2d(Constants.TOPRIGHT_X, Constants.TOPRIGHT_Y);
	private final Translation2d m_backLeftLoc = new Translation2d(Constants.BOTTOMLEFT_X, Constants.TOPLEFT_Y);
	private final Translation2d m_backRightLoc = new Translation2d(Constants.BOTTOMRIGHT_X, Constants.BOTTOMRIGHT_Y);
	private AHRS gyro = new AHRS();

	private final SwerveModule m_frontLeft = new SwerveModule(
			Constants.FRONTLEFT_DRIVE, 
			false,
			Constants.FRONTLEFT_PIVOT,
			Constants.TOPLEFT_ABS_ENCODER);
	private final SwerveModule m_frontRight = new SwerveModule(
			Constants.FRONTRIGHT_DRIVE,
			true,
			Constants.FRONTRIGHT_PIVOT,
			Constants.TOPRIGHT_ABS_ENCODER);
	private final SwerveModule m_backLeft = new SwerveModule(
			Constants.BACKLEFT_DRIVE,
			false,
			Constants.BACKLEFT_PIVOT,
			Constants.BOTTOMLEFT_ABS_ENCODER);
	private final SwerveModule m_backRight = new SwerveModule(
			Constants.BACKRIGHT_DRIVE,
			true,
			Constants.BACKRIGHT_PIVOT,
			Constants.BOTTOMRIGHT_ABS_ENCODER);

	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m_frontLeftLoc, m_frontRightLoc,
			m_backLeftLoc, m_backRightLoc);

	// getPosition is just placeholder for getting distance with encoders even
	// though wpilib uses it as an example
	// this took me like 30 min ot figure out
	// convert encoders to m

	private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, getHeading2d(),
			new SwerveModulePosition[] {
					m_frontLeft.getPosition(),
					m_frontRight.getPosition(),
					m_backLeft.getPosition(),
					m_backRight.getPosition()
			}, new Pose2d(0.0, 0.0, new Rotation2d()));

	public double setpoint;

	public DriveUtil() {
		setpoint = 0;

		resetGyro();
		calibrateGyro();
	}

	public void driveRobot(boolean fieldRelative) {
		var swerveModuleStates = kinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(  
								RobotContainer.getDriverLeftXboxX(),// * Constants.MAX_LINEAR_SPEED,
								RobotContainer.getDriverLeftXboxY(),// * Constants.MAX_LINEAR_SPEED,
								RobotContainer.getDriverRightXboxX(),// * Constants.MAX_ANGULAR_SPEED, 
								getHeading2d())
						: new ChassisSpeeds(RobotContainer.getDriverLeftXboxY() * Constants.MAX_LINEAR_SPEED,
								RobotContainer.getDriverLeftXboxX() * Constants.MAX_LINEAR_SPEED,//Note y and x swapped for first 2 arguments is not intuitive, x is "forward"
								RobotContainer.getDriverRightXboxX() * Constants.MAX_ANGULAR_SPEED));

		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_LINEAR_SPEED);

		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);
	}

	public void setSwerveModuleStates(SwerveModuleState[] states) {
		m_frontLeft.setDesiredState(states[0]);
		m_frontRight.setDesiredState(states[1]);
		m_backLeft.setDesiredState(states[2]);
		m_backRight.setDesiredState(states[3]);
	}

	public Rotation2d getHeading2d() {
		return Rotation2d.fromDegrees(gyro.getYaw());
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public double getHeading() {
		return gyro.getYaw();
	}

	public void resetGyro() {
		gyro.reset();
	}

	public void resetPose(Pose2d pose) {
		m_odometry.resetPosition(gyro.getRotation2d(), new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_backLeft.getPosition(),
			m_backRight.getPosition()
		}, pose);
	}

	public void calibrateGyro() {
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