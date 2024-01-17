// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.GrabUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpitCubeThenPush extends SequentialCommandGroup {
  /** Creates a new BottomGoalThenLeave. */
  public SpitCubeThenPush(GrabUtil gu, DriveUtil du, double time) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SpitSeconds(gu, time),
       new AutoFollowTrajectorySwerve(
        du, 
        PathPlanner.loadPath(
          "pushcube", 
          new PathConstraints(Constants.ALIGN_TO_TAG_MAX_VELOCITY, Constants.ALIGN_TO_TAG_MAX_ACCELERATION)
        ),
        new PIDController(Constants.AUTO_X_P, Constants.AUTO_X_I, Constants.AUTO_X_D),
        new PIDController(Constants.AUTO_Y_P, Constants.AUTO_Y_I, Constants.AUTO_Y_D),
        new PIDController(Constants.AUTO_THETA_P, Constants.AUTO_THETA_I, Constants.AUTO_THETA_D)
      )
    );
  }
}
