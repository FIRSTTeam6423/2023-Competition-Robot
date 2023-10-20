// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmUtil;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.GrabUtil;
import frc.robot.util.ArmState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoScoreAuto extends SequentialCommandGroup {
  /** Creates a new ToConeAuto. */
  public TwoScoreAuto(PathPlannerTrajectory path, DriveUtil du, GrabUtil gu, ArmUtil au) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    HashMap<String, Command> eventMap=new HashMap<>();
    eventMap.put("spit", new SpitSeconds(gu, 3));
    System.out.println("EAGHHHHHH"+path.getMarkers());
    addCommands(
      new SetArmState(au, ArmState.LOW),
      new SpitSeconds(gu, 1),
      new SetArmState(au, ArmState.RETRACT),
      new FollowPathWithEvents(
        new AutoFollowTrajectorySwerve(du, path),
        path.getMarkers(),
        eventMap
      ),
      new AutoAlignForScore(du) 
    );
  }
}
