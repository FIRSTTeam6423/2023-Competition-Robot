// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmUtil;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.GrabUtil;
import frc.robot.util.ArmState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BottomGoalThenLeave extends SequentialCommandGroup {
  /** Creates a new BottomGoalThenLeave. */
  public BottomGoalThenLeave(GrabUtil gu, DriveUtil du, ArmUtil au) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmPresetState(au, ArmState.LOW_GOAL),
      new SpitSeconds(gu, 1),
      new ParallelCommandGroup(
        new SetArmPresetState(au, ArmState.RETRACT),
        new AutoFollowTrajectorySwerve(du, PathPlanner.loadPath("Exit", new PathConstraints(
        Constants.ALIGN_TO_TAG_MAX_VELOCITY, Constants.ALIGN_TO_TAG_MAX_ACCELERATION)))
      )
      
    );
  }
}
