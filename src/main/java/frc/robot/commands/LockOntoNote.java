// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.SwerveModule;

public class LockOntoNote extends CommandBase {
  /** Creates a new DriveRobot. */
  public double deadzone(double input){
		if(Math.abs(input) >= Constants.XBOX_STICK_DEADZONE_WIDTH){
			return input;
		} else {
			return 0;
		}
	}
  
  public LockOntoNote(DriveUtil du) {
    // Use addRequirements() here to declare subsystem dependencies.
    int xSign = (int)Math.signum(RobotContainer.getDriverLeftXboxY());
		double xSpeed = xSign * Math.pow(deadzone(RobotContainer.getDriverLeftXboxY()), 2) 
						* Constants.MAX_LINEAR_SPEED 
						//* Math.cos(Math.toRadians(RobotContainer.allianceOrientation))
						* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); //reversed x and y so that up on controller is

		int ySign = (int)Math.signum(RobotContainer.getDriverLeftXboxX());
		double ySpeed = ySign * Math.pow(deadzone(RobotContainer.getDriverLeftXboxX()), 2) 
						* Constants.MAX_LINEAR_SPEED 
						//* Math.cos(Math.toRadians(RobotContainer.allianceOrientation))
						* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); //reversed x and y so that up on controller is

		double omega = deadzone(RobotContainer.getDriverRightXboxX()) 
						* Math.toRadians(Constants.MAX_ANGULAR_SPEED) 
						* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1);

    du.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega * -1, du.getHeading2d()));
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!RobotContainer.getDriverLeftBumper()) return true;
    return false;
  }
}