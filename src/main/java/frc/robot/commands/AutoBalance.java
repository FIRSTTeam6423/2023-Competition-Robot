// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveUtil;
import edu.wpi.first.wpilibj.Timer;


public class AutoBalance extends CommandBase {
  private DriveUtil du;
  private Timer timer;
  public AutoBalance(DriveUtil du) {
    this.du = du;
    addRequirements(this.du);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer=new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPitchDeg = du.getRoll();
    double error = -currentPitchDeg;

    double output = error * Constants.AUTO_BALANCE_P;
    if(Math.abs(error)<Constants.AUTO_BALANCE_DEADBAND){
      timer.start();
      output=0;
    }
    else {
      timer.reset();
    }
    du.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(output, 0, 0, du.getPose().getRotation()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get()>Constants.AUTO_BALANCE_TIME);
  }
}
