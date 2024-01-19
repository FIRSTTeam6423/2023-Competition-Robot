// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveUtil;
import frc.robot.util.IronUtil;

public class OperateDrive extends CommandBase {
  /** Creates a new OperateDrive. */
  private DriveUtil du;
  private boolean fieldRelative;

  public OperateDrive(DriveUtil du, boolean fieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.du = du;
	this.fieldRelative = fieldRelative;
	
    addRequirements(this.du);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    du.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
	double xInput = IronUtil.deadzone(RobotContainer.getDriverLeftXboxY(), Constants.XBOX_STICK_DEADZONE_WIDTH);
	double yInput = IronUtil.deadzone(RobotContainer.getDriverLeftXboxX(), Constants.XBOX_STICK_DEADZONE_WIDTH);
	double omegaInput = IronUtil.deadzone(RobotContainer.getDriverRightXboxX(), Constants.XBOX_STICK_DEADZONE_WIDTH);

	int xSign = (int)Math.signum(RobotContainer.getDriverLeftXboxY()); //Must keep sign because we are squaring input
	double xSpeed = xSign * Math.pow(xInput, 2)  //NEED TO REVERSE DEPENDING ON ALLIANCE COLOR
					* Constants.MAX_LINEAR_SPEED 
					* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); //reversed x and y so that up on controller is

	int ySign = (int)Math.signum(RobotContainer.getDriverLeftXboxX()); //Must keep sign because we are squaring input
	double ySpeed = ySign * Math.pow(yInput, 2)  //NEED TO REVERSES DEPENDING ON ALLIANCE COLOR
					* Constants.MAX_LINEAR_SPEED 
					* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); //reversed x and y so that up on controller is

	double omega =  omegaInput
					* Math.toRadians(Constants.MAX_ANGULAR_SPEED) 
					* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1);

	// var swerveModuleStates = kinematics.toSwerveModuleStates(
	// 		fieldRelative
	// 				? 
	// 				: new ChassisSpeeds(RobotContainer.getDriverLeftXboxY() * Constants.MAX_LINEAR_SPEED,
	// 						RobotContainer.getDriverLeftXboxX() * Constants.MAX_LINEAR_SPEED,//Note y and x swapped for first 2 arguments is not intuitive, x is "forward"
	// 						RobotContainer.getDriverRightXboxX() * Constants.MAX_ANGULAR_SPEED));

	ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
							xSpeed, //reversed x and y so that up on controller is
							ySpeed, //forward from driver pov
							omega, 
							du.getHeading2d());

	//SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_LINEAR_SPEED);

	du.setChassisSpeeds(speeds);
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