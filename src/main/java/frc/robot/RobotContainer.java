// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.OperateArm;
import frc.robot.commands.OperateDrive;
import frc.robot.commands.OperateGrab;
import frc.robot.subsystems.ArmUtil;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.GrabUtil;
import frc.robot.util.ArmState;
import frc.robot.subsystems.ClawUtil;
import frc.robot.commands.OperateClaw;
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
  private final ClawUtil clawUtil = new ClawUtil();
  private final GrabUtil grabUtil = new GrabUtil();

  private final OperateDrive operateDrive = new OperateDrive(driveUtil);
  private final OperateClaw operateClaw = new OperateClaw(clawUtil);
  private final OperateGrab operateGrab = new OperateGrab(grabUtil);

  private final ArmUtil armUtil = new ArmUtil();

  private final OperateArm operateArm = new OperateArm(armUtil);

  private static XboxController driver;
  private static XboxController operator;
  private JoystickButton clawButton;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private JoystickButton highButton;
  private JoystickButton middleButton;
  private JoystickButton lowButton;
  private JoystickButton highPButton;
  private JoystickButton groundPButton;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driver = new XboxController(Constants.XBOX_DRIVER);
    operator = new XboxController(Constants.XBOX_OPERATOR);


    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    highButton = new JoystickButton(operator, Button.kY.value);
    middleButton =  new JoystickButton(operator, Button.kX.value);
    lowButton = new JoystickButton(operator, Button.kA.value);
    highPButton = new JoystickButton(operator, Button.kLeftBumper.value);
    groundPButton = new JoystickButton(operator, Button.kRightBumper.value);
    clawButton = new JoystickButton(operator, Button.kB.value);

    highButton.onTrue(new InstantCommand(() -> armUtil.setState(ArmState.HIGH_GOAL), armUtil));
    middleButton.onTrue(new InstantCommand(() -> armUtil.setState(ArmState.MIDDLE_GOAL), armUtil));
    lowButton.onTrue(new InstantCommand(() -> armUtil.setState(ArmState.LOW_GOAL), armUtil));
    highPButton.onTrue(new InstantCommand(() -> armUtil.setState(ArmState.HIGH_PICK), armUtil));
    groundPButton.onTrue(new InstantCommand(() -> armUtil.setState(ArmState.GROUND_PICK), armUtil));

		clawButton.onTrue(new InstantCommand(() -> clawUtil.toggleClaw(), clawUtil));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  private void configureDefaultCommands(){
    driveUtil.setDefaultCommand(operateDrive);
    armUtil.setDefaultCommand(operateArm);
    clawUtil.setDefaultCommand(operateClaw);
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

  public static double getOperatorLeftXboxX(){
    return operator.getLeftX();
  }

  public static double getOperatorLeftXboxY(){
    return operator.getLeftY();
  }

  public static double getOperatorRightXboxX(){
    return operator.getRightX();
  }

  public static double getOperatorRightXboxY(){
    return operator.getRightY();
  }
}