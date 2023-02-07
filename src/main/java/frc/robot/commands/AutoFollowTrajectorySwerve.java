// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFollowTrajectorySwerve extends SequentialCommandGroup {
  /** Creates a new AutoFollowTrajectorySwerve. */
	public AutoFollowTrajectorySwerve(DriveUtil driveUtil, PathPlannerTrajectory traj) {
	// Add your commands in the addCommands() call, e.g.
	// addCommands(new FooCommand(), new BarCommand());
		addCommands(
			new PPSwerveControllerCommand(
            	traj, 
            	driveUtil::getPose, // Pose supplier
            	driveUtil.kinematics, // SwerveDriveKinematics
            	new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            	new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
            	new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            	driveUtil::setSwerveModuleStates, // Module states consumer
            	true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            	driveUtil // Requires this drive subsystem
        	)
		);

	}
}
