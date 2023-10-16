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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignToNearestGridTag;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoFollowTrajectorySwerve;
import frc.robot.commands.ExitAndBalance;
import frc.robot.commands.OperateDrive;
import frc.robot.commands.OperateGrab;
import frc.robot.commands.SpitCubeThenPush;
import frc.robot.commands.TwoScoreAuto;
import frc.robot.subsystems.ArmUtil;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.GrabUtil;
import frc.robot.util.WristState;
import frc.robot.commands.OperateArm;

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
  
  private static final GrabUtil grabUtil = new GrabUtil();
  private final OperateGrab operateGrab = new OperateGrab(grabUtil);

  private final OperateDrive operateDrive = new OperateDrive(driveUtil);

  private final ArmUtil armUtil = new ArmUtil();

  private final OperateArm operateArm = new OperateArm(armUtil);

  private static XboxController driver;
  private static Joystick operator;

  private static JoystickButton parallelToggleButton;
  private static JoystickButton retractWristButton;
  private static JoystickButton cargoRetractButton;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  public static double allianceOrientation = 0; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driver = new XboxController(Constants.XBOX_DRIVER);
    operator = new Joystick(Constants.JOYSTICK_OPERATOR);


   
    retractWristButton = new JoystickButton(operator, 4);
    parallelToggleButton = new JoystickButton(operator, 5);
    cargoRetractButton = new JoystickButton(operator, 6);
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();

    // autoChooser.setDefaultOption("Low goal arm state", new SetArmPresetState(armUtil, ArmState.LOW_GOAL));
    // autoChooser.setDefaultOption("Middle goal arm state", new SetArmPresetState(armUtil, ArmState.MIDDLE_GOAL));
    autoChooser.addOption("Align to grid tag", new AlignToNearestGridTag(driveUtil));
    // autoChooser.addOption("Bottom Goal Then Leave", new BottomGoalThenLeave(grabUtil, driveUtil, armUtil));
    autoChooser.addOption("Grab and move back", new AutoFollowTrajectorySwerve(driveUtil, 
      PathPlanner.loadPath("autoPickupAndAlign", new PathConstraints(
      Constants.ALIGN_TO_TAG_MAX_VELOCITY, Constants.ALIGN_TO_TAG_MAX_ACCELERATION))));
    autoChooser.addOption("align ot thing", new ExitAndBalance(driveUtil));
    autoChooser.addOption("spit 1 sec", new SpitCubeThenPush(grabUtil, driveUtil, 0.25));
    autoChooser.addOption("Balance Test", new AutoBalance(driveUtil));
    autoChooser.addOption("Two Cone Auto", new TwoScoreAuto(
      PathPlanner.loadPath("autoPickupAndAlign", new PathConstraints(Constants.ALIGN_TO_TAG_MAX_VELOCITY, Constants.ALIGN_TO_TAG_MAX_ACCELERATION)), 
      driveUtil, grabUtil, armUtil));
    SmartDashboard.putData("Autonomous Command", autoChooser);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    parallelToggleButton.onTrue(new InstantCommand(()->{
      armUtil.setWristState(WristState.PARALLEL_TO_GROUND);
    }));
    retractWristButton.onTrue(new InstantCommand(()->{
      armUtil.setWristState(WristState.RETRACTED);
    }));
    cargoRetractButton.onTrue(new InstantCommand(()->{
      armUtil.setWristState(WristState.CARGO_RETRACT);
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

  public static boolean getClawIntakeLimitSwitch() {
    return grabUtil.getIntakeLimitSwitch();
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