// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule extends SubsystemBase {
	/** Creates a new SwerveModule. */
	private static final int kEncoderResolution = 4096;
	private CANSparkMax driveMotor, pivotMotor;

	private RelativeEncoder driveEncoder;
	private AnalogEncoder pivotPosEncoder;

	// Gains are for example purposes only - must be determined for your own robot!
	private PIDController drivePIDController, pivotPIDController;

	private SwerveModuleState state;

	public SwerveModule(int driveMotorID, int pivotMotorID, int encoderID) {
		driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		pivotMotor = new CANSparkMax(pivotMotorID, MotorType.kBrushless);

		/**
		 * We need three encoders, as the sparkmax can only accurately tell
		 * the rpm of the motor, not its position. A third ecnoder needs to handle
		 * that and pass that in to the PIDController
		 **/
		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(kEncoderResolution);
		driveEncoder.setVelocityConversionFactor(Constants.METERS_PER_REV / 60);
		driveEncoder.setPosition(0);

		pivotPosEncoder = new AnalogEncoder(encoderID);
		pivotPosEncoder.setDistancePerRotation(Constants.RATIOED_COUNTS_PER_REV / 360);
		pivotPosEncoder.reset();

		drivePIDController = new PIDController(Constants.MODULEDRIVE_P, Constants.MODULEDRIVE_I, Constants.MODULEDRIVE_D);
		drivePIDController.setP(Constants.MODULEDRIVE_P);
		drivePIDController.setI(Constants.MODULEDRIVE_I);
		drivePIDController.setD(Constants.MODULEDRIVE_D);

		pivotPIDController = new PIDController(Constants.MODULEPIVOT_P, Constants.MODULEPIVOT_I, Constants.MODULEPIVOT_D);
		pivotPIDController.setP(Constants.MODULEPIVOT_P);
		pivotPIDController.setI(Constants.MODULEPIVOT_I);
		pivotPIDController.setD(Constants.MODULEPIVOT_D);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(pivotPosEncoder.getAbsolutePosition()));
	}

	public SwerveModulePosition getPosition() {
		Rotation2d r = new Rotation2d(pivotPosEncoder.getAbsolutePosition());
		return new SwerveModulePosition(driveEncoder.getPosition() / Constants.TICKS_PER_METER,
				r);
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		// Optimize the reference state to avoid spinning further than 90 degrees
		state = SwerveModuleState.optimize(desiredState, new Rotation2d(pivotPosEncoder.getDistance()));
		// Different constant need for drivePIDController, convert m/s to rpm
		driveMotor.set(drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond));
		// How to pass in encoder position from the analog encoder? Convert position to
		// velocity that is then passed in here?
		pivotMotor.set(pivotPIDController.calculate(pivotPosEncoder.getAbsolutePosition(), state.angle.getDegrees()));
	}

	public void stopModule() {
		driveMotor.set(0);
		pivotMotor.set(0);
	}

	public void resetEncoders() {
		driveEncoder.setPosition(0);
		pivotPosEncoder.reset();
	}

	@Override
	public void periodic() {
	}
}