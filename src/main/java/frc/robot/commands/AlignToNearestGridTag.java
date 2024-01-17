// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToNearestGridTag extends CommandBase {
  /** Creates a new AlignToNearestGridTag. */
  DriveUtil driveUtil;
  public AlignToNearestGridTag(DriveUtil driveUtil) {
    this.driveUtil=driveUtil;
  }

  @Override
  public void initialize(){
    PathPlannerTrajectory traj = new PathPlannerTrajectory();
    Pose2d robotPos = RobotContainer.getFieldPosed2dFromNearestCameraTarget();
    Pose3d tagPose = RobotContainer.getPose3dOfNearestCameraTarget();

    traj = PathPlanner.generatePath(
      new PathConstraints(Constants.ALIGN_TO_TAG_MAX_VELOCITY, Constants.ALIGN_TO_TAG_MAX_ACCELERATION),
      new PathPoint(
        robotPos.getTranslation(),
        new Rotation2d(tagPose.getRotation().getZ()),
        robotPos.getRotation() //Camera rotation not same as rogot
      ),
      new PathPoint(
        new Translation2d(tagPose.getX()+5, robotPos.getY()), //Change + or - depending on team color
        new Rotation2d(tagPose.getRotation().getZ()),
        new Rotation2d(tagPose.getRotation().getZ()
        + Math.PI)
      )
    );

    driveUtil.resetPose(traj.getInitialPose());

    //new AutoFollowTrajectorySwerve(driveUtil, traj).schedule();
  }
}
