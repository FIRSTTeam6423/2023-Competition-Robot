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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToPointWithTag extends CommandBase {
  /** Creates a new AlignToNearestGridTag. */
  DriveUtil driveUtil;
  PathPoint endPoint;
  public AlignToPointWithTag(DriveUtil driveUtil, PathPoint point) {
    endPoint = point;
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
      endPoint
    );
    
    SmartDashboard.putNumber("start X", traj.getInitialState().poseMeters.getX());
    SmartDashboard.putNumber("start Y", traj.getInitialState().poseMeters.getY());
    SmartDashboard.putNumber("start rot", traj.getInitialState().poseMeters.getRotation().getDegrees());

    SmartDashboard.putNumber("end X", traj.getEndState().poseMeters.getX());
    SmartDashboard.putNumber("end Y", traj.getEndState().poseMeters.getY());
    SmartDashboard.putNumber("end rot", traj.getEndState().poseMeters.getRotation().getDegrees());

    driveUtil.resetPose(traj.getInitialPose());

    new PPSwerveControllerCommand(
            	traj, 
            	driveUtil::getPose, // Pose supplier
            	driveUtil.kinematics, // SwerveDriveKinematics
            	new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            	new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
            	new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              //Only using feedforwards should be fine for this, we are already using pid in the set module states
            	driveUtil::setSwerveModuleStates, // Module states consumer
            	false, // This should always work regardless of alliance color
            	driveUtil // Requires this drive subsystem
        	).schedule();
  }
}
