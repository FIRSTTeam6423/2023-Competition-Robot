package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.util.ArmState;

public class ArmUtil {
    private CANSparkMax armMotor1, armMotor2;
    private ArmState state;
    private RelativeEncoder arm1Encoder, arm2Encoder;

    public ArmUtil() {
        armMotor1 = new CANSparkMax(Constants.ARM1, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(Constants.ARM2, MotorType.kBrushless);
        arm1Encoder = armMotor1.getEncoder();
        arm2Encoder = armMotor2.getEncoder();
    }

    public void resetEncoders() {
        arm1Encoder.setPositionConversionFactor(Constants.ENCODER_NUM);
        arm2Encoder.setPositionConversionFactor(Constants.ENCODER_NUM);
        arm1Encoder.setPosition(0);
        arm2Encoder.setPosition(0);
    }

    public void setState(ArmState newState) {
        state = newState;

    }

    // everything is in degrees
    public void turn(CANSparkMax motor, RelativeEncoder encoder, double degrees) {
        double error = degrees - encoder.getPosition();
        if (error > Constants.DEADBAND && error < -Constants.DEADBAND) {
            error = degrees - encoder.getPosition();
            motor.set(error * Constants.TURN_P_VALUE);
        }

    }

    public void OperateArm() {
        switch (state) {
            case STATE1:

                break;
            case STATE2:

                break;
            case STATE3:

                break;

        }
    }
}
