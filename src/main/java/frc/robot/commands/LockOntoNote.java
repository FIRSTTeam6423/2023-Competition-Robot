
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveUtil;

public class LockOntoNote extends CommandBase {
  /** Creates a new DriveRobot. */
  public PhotonCamera JohnCam = new PhotonCamera("johncam");
  private DriveUtil du;
  private PhotonTrackedTarget target;

  private double yaw;
  public double lockedRotation;

  public double deadzone(double input){
		if(Math.abs(input) >= Constants.XBOX_STICK_DEADZONE_WIDTH){
			return input;
		} else {
			return 0;
		}
	}
  
  public LockOntoNote(DriveUtil du) {
    this.du = du;

  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = JohnCam.getLatestResult();
    if (result.hasTargets() == false) return;
    target = result.getBestTarget();
    yaw = target.getYaw();
    SmartDashboard.putNumber("X note position", yaw);
    lockedRotation = (yaw <= 50) ? 25 : yaw * yaw * 0.01;
    SmartDashboard.putNumber("new rotation", lockedRotation);
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
    
    
    du.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, du.getHeading2d()));
  }

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