// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveUtil;
import frc.robot.util.SwerveController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFollowTrajectorySwerve extends CommandBase {
	//NEEEDS TO BE RENAMED TO AUTOFOLLOWPATHGROUP SOON
  /** Creates a new AutoFollowTrajectorySwerve. */
  	private DriveUtil du;
  	private PathPlannerTrajectory traj;
	private SwerveController holonomicController;

	//controlelrs
	PIDController xController;
	PIDController yController;
	PIDController thetaController;

	private Timer timer = new Timer();

	public AutoFollowTrajectorySwerve(
		DriveUtil du, 
		PathPlannerTrajectory traj, 
		PIDController xController, 
		PIDController yController, 
		PIDController thetaController
	) {
		this.du = du;
		this.traj = traj;
		this.xController=xController;
		this.yController=yController;
		this.thetaController = thetaController;

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		holonomicController = new SwerveController(xController, yController, thetaController);
	}

	@Override
 	public void initialize() {
		du.start();
		timer.reset();
		timer.start();

		PathPlannerState initialState = (PathPlannerState) traj.sample(0);
		Pose2d initialPose = new Pose2d(
			traj.getInitialPose().getTranslation(),
			initialState.holonomicRotation
		);

		System.out.println("END STATE: " + traj.getEndState().holonomicRotation);
		du.resetPose(initialPose);
  	}

	@Override
	public void execute() {
		PathPlannerState goal = (PathPlannerState) traj.sample(timer.get());
		//====NEED TO FLIP TRAJECTORY BASED ON ALLIANCE=====
        Rotation2d swerveRot;
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            swerveRot = new Rotation2d(
                -goal.holonomicRotation.getCos(),
    	        goal.holonomicRotation.getSin());
        } else {
            swerveRot = goal.holonomicRotation;
        }

		ChassisSpeeds speeds = holonomicController.calculate(
			du.getPose(), 
			goal.poseMeters, 
			goal.velocityMetersPerSecond, 
			swerveRot
		);
		
		du.setChassisSpeeds(speeds);
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
		//NEED AN END CONDITION
	}

	/*PIDController thetaController = new PIDController(.35, .035, 4.5);//new PIDController(4.5, 30, 5);
		//thetaController.enableContinuousInput(-Math.PI, Math.PI);
		addCommands(
			new InstantCommand(()->{
				
			}),
			new PPSwerveControllerCommand(
            	traj, 
            	driveUtil::getPose, // Pose supplier
				driveUtil.kinematics,
            	new PIDController(9, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            	new PIDController(9, 0, 0), // Y controller (usually the same values as X controller)
            	thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            	driveUtil::setSwerveModuleStates, // Module states consumer
            	false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            	driveUtil // Requires thuis drive subsystem
        	)
		);*/
}
