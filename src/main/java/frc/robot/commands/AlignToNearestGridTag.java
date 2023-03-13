// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToNearestGridTag extends CommandBase {
  /** Creates a new AlignToNearestGridTag. */
  DriveUtil driveUtil;
  Translation2d trans;
  Rotation2d head;
  Rotation2d holMot;
  public AlignToNearestGridTag(DriveUtil driveUtil, Translation2d trans, Rotation2d head, Rotation2d holMot) {
    this.driveUtil=driveUtil;
    this.trans=trans;
    this.head=head;
    this.holMot=holMot;
  }

  @Override
  public void initialize(){
    PathPlannerTrajectory traj = new PathPlannerTrajectory();
    Pose2d robotPos = RobotContainer.getFieldPosed2dFromNearestCameraTarget();
    Pose3d tagPose = RobotContainer.getPose3dOfNearestCameraTarget();
    SmartDashboard.putNumber("Robot X", robotPos.getX());
    SmartDashboard.putNumber("Robot Y", robotPos.getY());
    SmartDashboard.putNumber("Robot rot", robotPos.getRotation().getDegrees());

    traj = PathPlanner.generatePath(
      new PathConstraints(0.1, 0.5),
      new PathPoint(
        robotPos.getTranslation(),
        new Rotation2d(tagPose.getRotation().getZ()),
        robotPos.getRotation() //Camera rotation not same as rogot
      ),
      new PathPoint(
        new Translation2d(tagPose.getX()+1, robotPos.getY()),
        new Rotation2d(tagPose.getRotation().getZ()),
        new Rotation2d(tagPose.getRotation().getZ() + Math.PI)
      )
    );
    
    SmartDashboard.putNumber("end X", traj.getEndState().poseMeters.getX());
    SmartDashboard.putNumber("end Y", traj.getEndState().poseMeters.getY());
    SmartDashboard.putNumber("end rot", traj.getEndState().poseMeters.getRotation().getDegrees());
    driveUtil.resetPose(traj.getInitialPose());
    new PPSwerveControllerCommand(
            	traj, 
            	driveUtil::getPose, // Pose supplier
            	driveUtil.kinematics, // SwerveDriveKinematics
            	new PIDController(Constants.XDIR_P, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            	new PIDController(Constants.YDIR_P, 0, 0), // Y controller (usually the same values as X controller)
            	new PIDController(Constants.ROT_P, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            	driveUtil::setSwerveModuleStates, // Module states consumer
            	true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            	driveUtil // Requires this drive subsystem
        	).schedule();
  }
}
