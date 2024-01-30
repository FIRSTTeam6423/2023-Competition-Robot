// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveUtil;
import frc.robot.util.SwerveController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFollowTrajectorySwerve extends CommandBase {
	//TODO: NEEEDS TO BE RENAMED TO AUTOFOLLOWPATHGROUP SOON
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
		addRequirements(du);
	}

	@Override
 	public void initialize() {
		PathPlannerServer.startServer(5811);
		du.start();
		timer.reset();
		timer.start();

		Trajectory.State initialState = traj.sample(0);
		PathPlannerState ppStateInitial = (PathPlannerState) initialState;

		if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
			initialState = new Trajectory.State(
				initialState.timeSeconds, 
				initialState.velocityMetersPerSecond, 
				initialState.accelerationMetersPerSecondSq, 
				new Pose2d(
					Constants.FIELD_LENGTH_METERS - initialState.poseMeters.getX(),
					initialState.poseMeters.getY(),
					//new Rotation2d(
					//	-initialState.poseMeters.getRotation().getCos(),
					//	initialState.poseMeters.getRotation().getSin()
					//)
					initialState.poseMeters.getRotation()
				), 
				initialState.curvatureRadPerMeter //WAS NEGATED
			);
		}

		Rotation2d swerveRot;
		swerveRot = ppStateInitial.holonomicRotation;//.times(-1);
		if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
			    swerveRot = new Rotation2d(
			        -swerveRot.getCos(),
			        swerveRot.getSin()
				);
		}

		Pose2d initialPose = new Pose2d(
			initialState.poseMeters.getTranslation(),
			swerveRot
		);

		System.out.println("END STATE: " + traj.getEndState().holonomicRotation);
		du.resetPose(initialPose);
  	}

	@Override
	public void execute() {
		Trajectory.State goal = traj.sample(timer.get());
		PathPlannerState ppState = (PathPlannerState) goal;
		if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
			goal = new Trajectory.State(
				goal.timeSeconds, 
				goal.velocityMetersPerSecond, 
				goal.accelerationMetersPerSecondSq, 
				new Pose2d(
					Constants.FIELD_LENGTH_METERS - goal.poseMeters.getX(),
					goal.poseMeters.getY(),
					//new Rotation2d(
					//	-goal.poseMeters.getRotation().getCos(),
					//	goal.poseMeters.getRotation().getSin()
					//)
					goal.poseMeters.getRotation()
				), 
				goal.curvatureRadPerMeter //WAS NEGATED
			);
		}
		//====NEED TO FLIP TRAJECTORY BASED ON ALLIANCE=====
		
		PathPlannerServer.sendPathFollowingData(goal.poseMeters, du.getPose());

        Rotation2d swerveRot;
		swerveRot = ppState.holonomicRotation;//.times(-1);
		System.out.println("SWERVE BEFORE " + swerveRot.getDegrees());
		if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
			    swerveRot = new Rotation2d(
			        -swerveRot.getCos(),
			        swerveRot.getSin()
				);
		}
		System.out.println("SWERVE AFTER " + swerveRot.getDegrees());

		
		ChassisSpeeds speeds = holonomicController.calculate(
			du.getPose(), 
			goal.poseMeters, 
			goal.velocityMetersPerSecond, 
			swerveRot
		);
		
		SmartDashboard.putNumber("RUNNING GOAL", swerveRot.getDegrees());

		du.setChassisSpeeds(speeds);
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("COMMAND OVER EEE\n\n\n\n");
		du.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, du.getHeading2d()));
	}

	@Override
	public boolean isFinished() {
		double dist = du.getPose().getTranslation().getDistance(traj.getEndState().poseMeters.getTranslation());
		double angleErrorDegrees = Math.abs(du.getHeading2d().getDegrees() - traj.getEndState().holonomicRotation.getDegrees());

		if (timer.get() > traj.getTotalTimeSeconds() && dist < .1 && angleErrorDegrees < .1){
			return true;
		}
		

		SmartDashboard.putNumber("me", du.getPose().getRotation().getDegrees());
		SmartDashboard.putNumber("goal", traj.getEndState().holonomicRotation.getDegrees());
		return false;
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
