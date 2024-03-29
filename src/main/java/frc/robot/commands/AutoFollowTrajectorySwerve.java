// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFollowTrajectorySwerve extends SequentialCommandGroup {
  /** Creates a new AutoFollowTrajectorySwerve. */
	public AutoFollowTrajectorySwerve(DriveUtil driveUtil, PathPlannerTrajectory traj) {
	// Add your commands in the addCommands() call, e.g.
	// addCommands(new FooCommand(), new BarCommand());
		PIDController thetaController = new PIDController(.35, .035, 4.5);//new PIDController(4.5, 30, 5);
		//thetaController.enableContinuousInput(-Math.PI, Math.PI);
		addCommands(
			new InstantCommand(()->{
				driveUtil.start();
				PathPlannerState pInitialState = (PathPlannerState)traj.sample(0);
				Pose2d initialPose = new Pose2d(
					traj.getInitialPose().getTranslation(),
					pInitialState.holonomicRotation
				);
				System.out.println("END STATE THING: " + traj.getEndState().holonomicRotation);
				driveUtil.resetPose(initialPose);
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
		);

	}
}
