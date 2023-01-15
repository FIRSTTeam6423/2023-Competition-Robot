// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private static final int kEncoderResolution = 4096;

  private CANSparkMax driveMotor, pivotMotor;

  private RelativeEncoder driveEncoder, pivotEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private SparkMaxPIDController drivePIDController, pivotPIDController;

  private SwerveModuleState state;

  public SwerveModule(int driveMotorID, int pivotMotorID){
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(pivotMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(kEncoderResolution);
    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPositionConversionFactor(2 * Math.PI/kEncoderResolution);


    drivePIDController = driveMotor.getPIDController();
    drivePIDController.setP(Constants.MODULEDRIVE_P);
    drivePIDController.setI(Constants.MODULEDRIVE_I);
    drivePIDController.setD(Constants.MODULEDRIVE_D);

    pivotPIDController = pivotMotor.getPIDController();
    pivotPIDController.setP(Constants.MODULEPIVOT_P);
    pivotPIDController.setI(Constants.MODULEPIVOT_I);
    pivotPIDController.setD(Constants.MODULEPIVOT_D);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(pivotEncoder.getPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    state = SwerveModuleState.optimize(desiredState, new Rotation2d(pivotEncoder.getPosition()));
    driveMotor.set(state.speedMetersPerSecond/Constants.MAX_LINEAR_SPEED);
    pivotPIDController.setReference(state.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
  }


  public void resetEncoders(){
    driveEncoder.setPosition(0);
    pivotEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
  }
}