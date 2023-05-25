// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmUtil;
import frc.robot.util.ArmState;

public class SetArmPresetState extends CommandBase {
  private ArmUtil au;
  private ArmState armState;

  private double desiredArmAngle = 15;
  /** Creates a new SetArmPresetState. */
  public SetArmPresetState(ArmUtil au, ArmState armState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.au = au;
    this.armState = armState;
    switch(this.armState) {
      case CONTROL:
      case INITIALIZE:
      case HIGH_PICK:
        break;
      case HIGH_GOAL:
        desiredArmAngle = Constants.HIGH_GOAL_ARM;
        break;
      case MIDDLE_GOAL:
        desiredArmAngle = Constants.MIDDLE_GOAL_ARM;
        break;
        case LOW_GOAL:
        desiredArmAngle = Constants.LOW_GOAL_ARM;
        break;
      case GROUND_PICK:
        desiredArmAngle = Constants.GROUND_PICK_ARM;
        break;
    }
    addRequirements(this.au);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // au.setArmState(armState);
    System.out.println(desiredArmAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // au.operateArm(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(au.getArmAngleRelativeToGround() - desiredArmAngle) < 5);
  }
}
