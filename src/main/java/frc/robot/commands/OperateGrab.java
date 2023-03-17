// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabUtil;

public class OperateGrab extends CommandBase {
  private GrabUtil gu;
  /** Creates a new OperateGrab. */
  public OperateGrab(GrabUtil gu) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gu = gu;
    addRequirements(this.gu);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gu.operateGrabber();
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
