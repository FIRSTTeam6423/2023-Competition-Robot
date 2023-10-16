package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.ArmState;
import frc.robot.util.WristState;

public class ArmUtil extends SubsystemBase{
    private WristState wristState;
    private ArmState armState;

    private CANSparkMax armMotor1, armMotor2, wristMotor;

    private PIDController armPIDController, wristPIDController; //both positional pid controllers
    private ArmFeedforward armFeedForwardController, armFeedForwardControllerClawOut, wristFeedForwardController;

    private RelativeEncoder arm1Encoder, arm2Encoder, wristEncoder;
    private DigitalInput armLimitSwitch, wristLimitSwitch;

    private TrapezoidProfile armProfile, wristProfile;

    private double armSetpointDeg = -75;    //Setpoint of arm relative to the ground
    private double armStateTimeElapsed = 0; //Time elapsed since last arm state change

    private double wristStateTimeElapsed = 0;

    public ArmUtil() {
        //Trapezoid Profile for preset states
        armProfile = new TrapezoidProfile(
            new Constraints(0, 0), 
            new State(0, 0)
        );

        wristProfile = new TrapezoidProfile(
            new Constraints(0, 0), 
            new State(0,0)
        );

        armState = ArmState.INITIALIZE;
        wristState = WristState.RETRACTED;

        initMotors();

        initControlSystems();        

        //====Sensors====//
        armLimitSwitch = new DigitalInput(Constants.ARM_LIMIT_SWITCH);  
        wristLimitSwitch = new DigitalInput(Constants.WRIST_LIMIT_SWITCH);
    }

    private void initControlSystems() {
        //====Control Systems====//
        armPIDController = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);
        wristPIDController = new PIDController(Constants.WRIST_P, Constants.WRIST_I, Constants.WRIST_D);
        
        armFeedForwardController = new ArmFeedforward(
            Constants.ARM_kS, 
            Constants.ARM_kG/12, 
            Constants.ARM_kV, 
            Constants.ARM_kA
        ); //is the kg/12 because volts and using set vs setvolatage?
        
        armFeedForwardControllerClawOut = new ArmFeedforward(
            Constants.ARM_kS, 
            Constants.ARM_kG_CLAW_OUT/12, 
            Constants.ARM_kV, 
            Constants.ARM_kA
        ); 
        
        wristFeedForwardController = new ArmFeedforward(
            Constants.WRIST_kS, 
            Constants.WRIST_kG, 
            Constants.WRIST_kV, 
            Constants.WRIST_kA
        );
    }

    private void initMotors() {
        armMotor1 = new CANSparkMax(Constants.ARM1, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(Constants.ARM2, MotorType.kBrushless);
        wristMotor = new CANSparkMax(Constants.WRIST,MotorType.kBrushless);
       
        wristMotor.setIdleMode(IdleMode.kBrake);

        wristMotor.setIdleMode(IdleMode.kBrake);
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor2.setIdleMode(IdleMode.kBrake);

        armMotor1.setInverted(true); //up is positive for both arm and wrist

        arm1Encoder = armMotor1.getEncoder();
        arm2Encoder = armMotor2.getEncoder();
        wristEncoder = wristMotor.getEncoder();

        arm1Encoder.setPositionConversionFactor(Constants.ARM_CONVERSION_FACTOR); //Now fixed. Problem was we were assuming native units were
        arm2Encoder.setPositionConversionFactor(Constants.ARM_CONVERSION_FACTOR); //ticks when its actually rotations. Constants show fix
        
        arm1Encoder.setVelocityConversionFactor(Constants.ARM_CONVERSION_FACTOR/60);
        arm2Encoder.setVelocityConversionFactor(Constants.ARM_CONVERSION_FACTOR/60);
        
        wristEncoder.setPositionConversionFactor(Constants.WRIST_CONVERSION_FACTOR); //Me personally, i would test moving arm on own, then add on wrist once code to make sure it doesnt hit bumper exists
    }

    public void resetEncoders() {
        arm1Encoder.setPosition(0);
        arm2Encoder.setPosition(0);
        wristEncoder.setPosition(0);
    }

    public void setArmState(ArmState newState) {
        if(armState == ArmState.INITIALIZE) return;
        armState = newState;
        switch(armState) {
            case INITIALIZE:
            case RETRACT:
            case CONTROL:
                break;
            case HIGH_GOAL:
                armProfile = new TrapezoidProfile( //Note: high goal arm and initial position both have to assume same angle is zero
                    Constants.ARM_PROFILE_CONSTRAINTS, 
                    new State(Constants.HIGH_GOAL_ARM, 0),
                    new State(getArmAngleRelativeToGround(),0)
                );
                break;
            case MIDDLE_GOAL: 
                armProfile = new TrapezoidProfile( //Note: middle goal arm and initial position both have to assume same angle is zero
                    Constants.ARM_PROFILE_CONSTRAINTS, 
                    new State(Constants.MIDDLE_GOAL_ARM, 0),
                    new State(getArmAngleRelativeToGround(),0)
                );
                break;
            case LOW: 
                armProfile = new TrapezoidProfile( //Note: middle goal arm and initial position both have to assume same angle is zero
                    Constants.ARM_PROFILE_CONSTRAINTS, 
                    new State(Constants.LOW_GOAL_ARM, 0),
                    new State(getArmAngleRelativeToGround(),0)
                );
                break;
        }
        armStateTimeElapsed = 0;
    }

    public void setWristState(WristState state) {
        if (armState == ArmState.INITIALIZE) return;
        wristState = state;
        switch(state) {
            case CARGO_RETRACT:
                wristProfile = new TrapezoidProfile( //Note: high goal arm and initial position both have to assume same angle is zero
                    Constants.WRIST_PROFILE_CONSTRAINTS, 
                    new State(75, 0),
                    new State(getWristAngleRelativeToGround(),0)
                );
                break;
            case PARALLEL_TO_GROUND:
                wristProfile = new TrapezoidProfile( //Note: high goal arm and initial position both have to assume same angle is zero
                    Constants.WRIST_PROFILE_CONSTRAINTS, 
                    new State(-13, 0),
                    new State(getWristAngleRelativeToGround(),0)
                );
                break;
            
            case OUT:

                break;
            case RETRACTED:

                break;
        }
        wristStateTimeElapsed = 0;
    }

    /**
     * setArmVelocity will set the arm's velocity - should be used for operator manual control
    */
    public void setArmVelocity(double degPerSec){ //I'm not sure if you can use the same pid for velocity and position
        System.out.println("thing" + armPIDController.calculate(arm1Encoder.getVelocity(), degPerSec));

        armMotor1.set(
            MathUtil.clamp(armFeedForwardController.calculate(
                Math.toRadians(getArmAngleRelativeToGround()), 0),
                //+ armPIDController.calculate(arm1Encoder.getVelocity(), degPerSec),
                -0.5,
                0.5
            )
        );
        armMotor2.set(
            MathUtil.clamp(armFeedForwardController.calculate(
                Math.toRadians(getArmAngleRelativeToGround()), 0),
                //+ armPIDController.calculate(arm1Encoder.getVelocity(), degPerSec),
                -0.5, 
                0.5
            )
        );
    }

    /**
     * Moves arm down towards limit switch
     * 
     * @return limit switch triggered
     */
    public boolean operateArmToLimitSwitch() {
        if(!armLimitSwitch.get()) {
            armMotor1.set(0);
            armMotor2.set(0);
            return true;
        }
        armMotor1.set(-.05);
        armMotor2.set(-.05);
        return false;
    }

    /**
     * Used to attain a certain preset state. Uses motion profile with pid and feedforward kG
    */
    public void operateArmPreset() {
        //feedforward controller with only kg
        //pid controller for arm position
        //wristState = WristState.PARALLEL_TO_GROUND;
        //operateWrist();
        State setpoint = armProfile.calculate(armStateTimeElapsed); // should generate a nice curve for our position - we don't care about velocity because we cannot
        // work with velocity, as we are not able to find kV efficiently. PID should handle velocity nicely
        SmartDashboard.putNumber("setpoint", setpoint.position);
        armMotor1.set(
            MathUtil.clamp(armFeedForwardController.calculate(
                Math.toRadians(setpoint.position), 0)
                + armPIDController.calculate(getArmAngleRelativeToGround(), setpoint.position),
                -0.3,
                0.4
            )
        );
        armMotor2.set(
            MathUtil.clamp(armFeedForwardController.calculate(
                Math.toRadians(setpoint.position), 0)
                + armPIDController.calculate(getArmAngleRelativeToGround(), setpoint.position),
                -0.3,
                0.4
            )
        );
        if(Math.abs(armProfile.calculate(armProfile.totalTime()).position - getArmAngleRelativeToGround()) < 5) {
            wristState = WristState.PARALLEL_TO_GROUND;
            System.out.println("bruh!!!!");
        } else {
            wristState = WristState.RETRACTED;
        }
        //operateWrist();
    }


    /**
     * 0 degrees is parallel to the x axis, 
     * and a positive increase in angle is
     * raising the arm.

     * @return arm angle relative to the horizontal
     */
    public double getArmAngleRelativeToGround(){
        return arm1Encoder.getPosition() - 82;
    }

    public double getWristAngleRelativeToGround() {
        return arm1Encoder.getPosition() + wristEncoder.getPosition() + 78;
    }

    public void setWristAngleRelativeToGround(double degrees) {
        //degrees is angle with 0 being parallel to x axis, positive increase go towards heaven
        double curWristDegsRelGround = arm1Encoder.getPosition() + wristEncoder.getPosition() + 78;
        wristMotor.set(
            MathUtil.clamp(
                wristFeedForwardController.calculate(
                    Math.toRadians(degrees),
                    0
                ) + wristPIDController.calculate(
                    curWristDegsRelGround, 
                    degrees
                ),
                -1,
                !wristLimitSwitch.get() ? 0 : 1
            )
        );
        
    }

    public void operateArm(double joystickInput) {
        switch(armState) {
            case INITIALIZE: //In initialize, the user cannot move the arm and it is zeroing itself.
                boolean armAtLimitSwitch = operateArmToLimitSwitch();
                boolean wristAtLimitSwitch = operateWristToLimitSwitch();
                if(armAtLimitSwitch && wristAtLimitSwitch){
                    armState = ArmState.CONTROL;
                    wristState = WristState.RETRACTED;
                }
                break;
            case RETRACT: //Arm is being brought down, but will be controllable when user moves joystick
                operateArmToLimitSwitch();
                if(joystickInput > Constants.ARM_JOYSTICK_INPUT_DEADBAND) armState = ArmState.CONTROL;
                break;
            case CONTROL: //In control, the user has full control over the arm.
                double input = joystickInput;
                double deltaSetpoint = (input * 75) / 50; // change in setpoint per periodic tick

                if(armSetpointDeg + deltaSetpoint > 10) {
                    armSetpointDeg = 10;
                } else if (armSetpointDeg + deltaSetpoint < -75) {
                    if((wristState == WristState.CARGO_RETRACT || wristState == WristState.RETRACTED) && joystickInput < 0) {
                        //setArmState(ArmState.RETRACT);
                        armSetpointDeg = -75;
                    }
                } else {
                    armSetpointDeg += deltaSetpoint;
                }

                double feeedForwardValue = RobotContainer.getClawIntakeLimitSwitch() == false //wristState == WristState.PARALLEL_TO_GROUND 
                ? armFeedForwardControllerClawOut.calculate(Math.toRadians(armSetpointDeg), 0)  
                : armFeedForwardController.calculate(Math.toRadians(armSetpointDeg), 0);

                //System.out.println("Setpoint: " + armSetpointDeg);
                if(armLimitSwitch.get()){ //limit switch is not triggered
                    armMotor1.set(
                        feeedForwardValue
                        + armPIDController.calculate(getArmAngleRelativeToGround(), armSetpointDeg)
                    );
                    armMotor2.set(
                        feeedForwardValue
                        + armPIDController.calculate(getArmAngleRelativeToGround(), armSetpointDeg)
                    );
                } else {
                    armMotor1.set(0);
                    armMotor2.set(0);
                }

                operateWrist();

                break;
            case HIGH_GOAL: //preset for high goal
            case MIDDLE_GOAL: //preset for middle goal
            case LOW: // preset for the low goal
        }
    }

    public boolean operateWristToLimitSwitch(){
        if(!wristLimitSwitch.get()) {
            wristMotor.set(0);
            return true;
        }
        wristMotor.set(0.25);
        return false;
    }

    public void lowerWrist(){
        if(wristEncoder.getPosition() <= Constants.WRIST_PHYSICAL_STOP + Constants.WRIST_STOP_DEADBAND) {
            wristMotor.set(0);
        } else {
            wristMotor.set(-.2);
        }
    }

    public void operateWrist() {
        State setpoint = wristProfile.calculate(wristStateTimeElapsed);
        switch(wristState) {
            case RETRACTED:
                operateWristToLimitSwitch();
                break;
            case CARGO_RETRACT:
                setWristAngleRelativeToGround(setpoint.position);
                //setWristAngleRelativeToGround(75);
                break;
            case OUT:
                lowerWrist();
                break;
            case PARALLEL_TO_GROUND:
                setWristAngleRelativeToGround(setpoint.position); // "0" should be parallel to the ground, offset slightly to make intake easier
                //setWristAngleRelativeToGround(-15);
                break;
        }
    }

    public void periodic() {
        armStateTimeElapsed += .02; //assumes periodic rate is 20ms = .02s
        wristStateTimeElapsed += .02;
        if(!armLimitSwitch.get()){
            arm1Encoder.setPosition(0);
            arm2Encoder.setPosition(0);
        }
        if(!wristLimitSwitch.get()){
            wristEncoder.setPosition(0);
        }

        SmartDashboard.putNumber("Wrist Encoder", wristEncoder.getPosition());
        SmartDashboard.putBoolean("Wrist Switch", wristLimitSwitch.get());
        SmartDashboard.putBoolean("Arm Switch", armLimitSwitch.get());
        SmartDashboard.putNumber("Wrist Relative Angle", arm1Encoder.getPosition() + wristEncoder.getPosition() + 99);
        SmartDashboard.putNumber("arm angle", getArmAngleRelativeToGround());
    }
}