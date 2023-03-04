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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Currency;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule extends SubsystemBase {
	/** Creates a new SwerveModule. */
	private static final int kEncoderResolution = 4096;
	private CANSparkMax driveMotor, pivotMotor;

	private RelativeEncoder driveEncoder;
	private DutyCycleEncoder pivotEncoder;

	// Gains are for example purposes only - must be determined for your own robot!
	private PIDController drivePIDController, pivotPIDController;

	private int encoderID;

	private SwerveModuleState state;

	public SwerveModule(int driveMotorID, boolean driveInverted, int pivotMotorID, int encoderID) {
		driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		driveMotor.setInverted(driveInverted);
		pivotMotor = new CANSparkMax(pivotMotorID, MotorType.kBrushless);
		pivotMotor.setInverted(true);
		this.encoderID = encoderID;
		/**
		 * We need three encoders, as the sparkmax can only accurately tell
		 * the rpm of the motor, not its position. A third ecnoder needs to handle
		 * that and pass that in to the PIDController
		 **/
		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(kEncoderResolution);
		driveEncoder.setVelocityConversionFactor(Constants.METERS_PER_REV / 60);
		driveEncoder.setPosition(0);

		pivotEncoder = new DutyCycleEncoder(encoderID);
		pivotEncoder.reset();

		drivePIDController = new PIDController(Constants.MODULEDRIVE_P, Constants.MODULEDRIVE_I, Constants.MODULEDRIVE_D);
		drivePIDController.setP(Constants.MODULEDRIVE_P);
		drivePIDController.setI(Constants.MODULEDRIVE_I);
		drivePIDController.setD(Constants.MODULEDRIVE_D);

		pivotPIDController = new PIDController(Constants.MODULEPIVOT_P, Constants.MODULEPIVOT_I, Constants.MODULEPIVOT_D);
		pivotPIDController.enableContinuousInput(-90, 90);
		pivotPIDController.setP(Constants.MODULEPIVOT_P);
		pivotPIDController.setI(Constants.MODULEPIVOT_I);
		pivotPIDController.setD(Constants.MODULEPIVOT_D);

		state = getState();
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(pivotEncoder.getAbsolutePosition()));
	}

	public SwerveModulePosition getPosition() {
		Rotation2d r = new Rotation2d(pivotEncoder.getAbsolutePosition());
		return new SwerveModulePosition(driveEncoder.getPosition() / Constants.TICKS_PER_METER,
				r);
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		// Optimize the reference state to avoid spinning further than 90 degrees
		double curRotDeg = pivotEncoder.getAbsolutePosition() * 360 - Constants.ABS_ENCODER_OFFSETS[this.encoderID];//-pivotEncoder.getAbsolutePosition() * 360 - Constants.ABS_ENCODER_OFFSETS[this.encoderID];
		state = SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(curRotDeg)));
		// Different constant need for drivePIDController, convert m/s to rpm
		driveMotor.set(drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond));
		pivotMotor.set(pivotPIDController.calculate(curRotDeg,state.angle.getDegrees()));
		//SmartDashboard.putNumber("Desired Angle id: " + encoderID , state.angle.getDegrees());
		SmartDashboard.putNumber("curRotDeg: " + encoderID, curRotDeg);
		SmartDashboard.putNumber("pid output: " + encoderID, pivotPIDController.calculate(curRotDeg, 0));
		
		
		//SmartDashboard.putNumber("ABS encoder" + Integer.toString(this.encoderID), pivotEncoder.getAbsolutePosition() * 360 + Constants.ABS_ENCODER_OFFSETS[this.encoderID]);
	}

	public void stopModule() {
		driveMotor.set(0);
		pivotMotor.set(0);
	}

	public void resetEncoders() {
		driveEncoder.setPosition(0);
		pivotEncoder.reset();
	}

	public double pivotValue(){
		return pivotEncoder.getAbsolutePosition();
	}

	public double drivePosValue(){
		return driveEncoder.getPosition();
	}

	public double driverVelValue(){
		return driveEncoder.getVelocity();
	}

	public double stateSpeed(){
		return state.speedMetersPerSecond;
	}

	public double stateAngle(){
		return state.angle.getDegrees();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Current Angle" + encoderID , pivotEncoder.getAbsolutePosition());
		SmartDashboard.putNumber("Current Angle deg" + encoderID , pivotEncoder.getAbsolutePosition()*360);
	}
}