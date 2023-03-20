// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignToNearestGridTag;
import frc.robot.commands.AutoFollowTrajectorySwerve;
import frc.robot.commands.BottomGoalThenLeave;
import frc.robot.commands.OperateArm;
import frc.robot.commands.OperateDrive;
import frc.robot.commands.OperateGrab;
import frc.robot.subsystems.ArmUtil;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.GrabUtil;
import frc.robot.util.ArmState;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveUtil driveUtil = new DriveUtil();
  private static final PhotonCamera camera = new PhotonCamera("johncam");
  
  private final GrabUtil grabUtil = new GrabUtil();
  private final OperateGrab operateGrab = new OperateGrab(grabUtil);

  private final OperateDrive operateDrive = new OperateDrive(driveUtil);

  private final ArmUtil armUtil = new ArmUtil();

  private final OperateArm operateArm = new OperateArm(armUtil);

  private static XboxController driver;
  private static Joystick operator;
  private static JoystickButton middleGoalButton;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  public static double allianceOrientation = 0; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driver = new XboxController(Constants.XBOX_DRIVER);
    operator = new Joystick(Constants.JOYSTICK_OPERATOR);


    middleGoalButton = new JoystickButton(operator, 9);

    autoChooser.setDefaultOption("align to grid", new AlignToNearestGridTag(driveUtil));

    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();

    autoChooser.setDefaultOption("Bottom Goal Then Leave", new BottomGoalThenLeave(grabUtil, driveUtil));
    autoChooser.setDefaultOption("Leave Then Balance", new AutoFollowTrajectorySwerve(driveUtil, 
    PathPlanner.loadPath("Exit", new PathConstraints(
      Constants.ALIGN_TO_TAG_MAX_VELOCITY, Constants.ALIGN_TO_TAG_MAX_ACCELERATION))));

    SmartDashboard.putData("Autonomous Command", autoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    middleGoalButton.onTrue(new InstantCommand(()->{
      armUtil.setArmState(ArmState.MIDDLE_GOAL);
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
		return autoChooser.getSelected();
  }

  private void configureDefaultCommands(){
    driveUtil.setDefaultCommand(operateDrive);
    armUtil.setDefaultCommand(operateArm);
    grabUtil.setDefaultCommand(operateGrab);
  }

  public static double getDriverLeftXboxX(){
    return driver.getLeftX();
  }

  public static double getDriverLeftXboxY(){
    return driver.getLeftY();
  }

  public static double getDriverRightXboxX(){
    return driver.getRightX();
  }

  public static double getDriverRightXboxY(){
    return driver.getRightY();
  }

  public static double getDriverLeftXboxTrigger(){
    return driver.getLeftTriggerAxis();
  }

  public static double getDriverRightXboxTrigger(){
    return driver.getRightTriggerAxis();
  }

  public static boolean getDriverAButton(){
    return driver.getAButton();
  }

  public static boolean getDriverBButton(){
    return driver.getBButton();
  }

  public static boolean getDriverXButton(){
    return driver.getXButton();
  }

  public static boolean getDriverYButton(){
    return driver.getYButton();
  }

  public static boolean getDriverLeftBumper(){
    return driver.getLeftBumper();
  }

  public static boolean getDriverRightBumper(){
    return driver.getRightBumper();
  }

  public static boolean getDriverLeftStickButton(){
    return driver.getLeftStickButton();
  }  

  public static boolean getDriverRightStickButton(){
    return driver.getRightStickButton();
  } 

  public static double getOperatorJoystickX(){
    return operator.getX();
  }

  public static double getOperatorJoystickY(){
    return operator.getY();
  }

  public static double getOperatorSlider(){
    return operator.getThrottle();
  }

  public static boolean getOperatorButton(int num){
    return operator.getRawButton(num);
  }

  public static boolean getOperator2Button(){
    return operator.getRawButton(2);
  }

  public static boolean getOperator3Button(){
    return operator.getRawButton(3);
  }

  public static boolean getOperator4Button(){
    return operator.getRawButton(4);
  }

  public static boolean getOperator5Button(){
    return operator.getRawButton(5);
  }

  public static boolean getOperator6Button(){
    return operator.getRawButton(6);
  }

  public static boolean getOperator7Button(){
    return operator.getRawButton(7);
  }

  public static boolean getOperator8Button(){
    return operator.getRawButton(8);
  }

  public static boolean getOperator9Button(){
    return operator.getRawButton(9);
  }

  public static boolean getOperator10Button(){
    return operator.getRawButton(10);
  }

  public static boolean getOperator11Button(){
    return operator.getRawButton(11);
  }

  public static boolean getOperator12Button(){
    return operator.getRawButton(12);
  }

  public static Pose3d getTagPose3dFromId(int id) {
		return Constants.TagPoses[id - 1];
	}

	public static PhotonTrackedTarget getNearestCameraTarget() {
		PhotonPipelineResult result = camera.getLatestResult();
		if (result.hasTargets()) {
			return result.getBestTarget();
		}
		return null;
	}

	public static List<PhotonTrackedTarget> getAllCameraTargets() {
		PhotonPipelineResult result = camera.getLatestResult();
		if (result.hasTargets()) {
			return result.getTargets();
		} else {
			return Collections.<PhotonTrackedTarget>emptyList();
		}
	}

	public static Pose3d getPose3dOfNearestCameraTarget() {
		PhotonPipelineResult result = camera.getLatestResult();
		if (result.hasTargets()) {
			PhotonTrackedTarget target = result.getBestTarget();
			Pose3d tagPose = getTagPose3dFromId(target.getFiducialId());
			return tagPose;
		}
    DriverStation.reportWarning("No nearest camera target to get Pose3d!", false);
		return null;
	}

	public static Pose2d getFieldPosed2dFromNearestCameraTarget() {
		PhotonPipelineResult result = camera.getLatestResult();
		if (result.hasTargets()) {
			PhotonTrackedTarget target = result.getBestTarget();
			Pose3d tagPose = getTagPose3dFromId(target.getFiducialId());
			Pose3d pos = PhotonUtils.estimateFieldToRobotAprilTag(
          target.getBestCameraToTarget(),   
          tagPose,
          Constants.CAMERA_TO_ROBOT // TODO: ADD THIS
					);
      allianceOrientation = Math.toDegrees(tagPose.getRotation().getZ());
			return new Pose2d(
					pos.getX(),
					pos.getY(),
					new Rotation2d(pos.getRotation().getZ()));
		}
		DriverStation.reportWarning("Could not get Pose2d from camera target: no targets found.", false);
		return null;
	}
}