// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.GrabberState;

public class GrabUtil extends SubsystemBase {
  private CANSparkMax grabMotor;
  private RelativeEncoder grabEncoder;
  private GrabberState state=GrabberState.OFF;
  /** Creates a new GrabUtil. */
  public GrabUtil() {
    grabMotor = new CANSparkMax(Constants.GRAB_MOTOR, MotorType.kBrushless);
    grabEncoder = grabMotor.getEncoder(); 
  }
  
  public void setState(GrabberState newState){
    state = newState;
  }

  public void operateGrabber(boolean intake, boolean spitting){
    switch(state){
      case OFF:
        grabMotor.set(0);
        if(intake){
          state = GrabberState.INTAKE;
        }
        if(spitting){
          state =  GrabberState.OUTPUT;
        }
        break;
      case INTAKE:
        grabMotor.set(Constants.GRAB_INTAKE_SPEED);
        if(!intake){
          state = GrabberState.OFF;
        }
        break;
      case OUTPUT:
        grabMotor.set(Constants.GRAB_OUTPUT_SPEED);
        if(!spitting){
          state = GrabberState.OFF;
        }
        break;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
