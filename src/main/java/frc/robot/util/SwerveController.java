package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveController {
    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;

    public SwerveController(
        PIDController xController,
        PIDController yController, 
        PIDController thetaController
    ) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ChassisSpeeds calculate(Pose2d curPose, Pose2d goalPose, double goalLinearVelocity, Rotation2d goalAngle) {
        Rotation2d curRot = curPose.getRotation();
        double xFeedForward = goalLinearVelocity * curRot.getCos();
        double yFeedForward = goalLinearVelocity * curRot.getSin();

        double xFeedback = xController.calculate(curPose.getX(), goalPose.getX());
        double yFeedback = yController.calculate(curPose.getY(), goalPose.getY());
        double thetaFeedback = thetaController.calculate(curRot.getRadians(), goalAngle.getRadians());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedForward + xFeedback, 
            yFeedForward + yFeedback, 
            thetaFeedback,
            curRot
        );
    }
}
