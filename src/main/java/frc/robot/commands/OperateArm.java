// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmUtil;

public class OperateArm extends CommandBase {
  /** Creates a new OperateDrive. */
  private ArmUtil au;
  private RetractArm initializeCommand;
  private boolean wristZeroed;
  private boolean armZeroed;

  public OperateArm(ArmUtil au) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.au = au;
    addRequirements(this.au);
    System.out.println("IIIIIIIIIIIIIIIIIIIIIIIIIIIiii");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //initializeCommand = new RetractArm(au);
    //System.out.println("EEEEEEEEEEEEEEEEE");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      au.operateArmStates();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}