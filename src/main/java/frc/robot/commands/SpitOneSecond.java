// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabUtil;
import frc.robot.util.GrabberState;

import java.util.Timer;

public class SpitOneSecond extends CommandBase {
  /** Creates a new SpitOneSecond. */
  private double ticker = 0;
  private GrabUtil gu;

  public SpitOneSecond(GrabUtil gu) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gu = gu;
    addRequirements(this.gu);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gu.setState(GrabberState.OUTPUT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ticker += .02; // 20 ms rate
    gu.operateGrabber();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gu.setState(GrabberState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ticker >=1;
  }
}
