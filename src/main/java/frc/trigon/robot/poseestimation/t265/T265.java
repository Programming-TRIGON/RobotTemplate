package frc.trigon.robot.poseestimation.t265;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import org.littletonrobotics.junction.Logger;

public class T265 {
    private final NetworkTable t265NetworkTable = NetworkTableInstance.getDefault().getTable("T265");

    private final IntegerSubscriber framesPerSecond = t265NetworkTable.getIntegerTopic("FPS").subscribe(0);
    private final DoubleSubscriber batteryPercentage = t265NetworkTable.getDoubleTopic("BatteryPercentage").subscribe(0.0f);
    private final FloatArraySubscriber positionMeters = t265NetworkTable.getFloatArrayTopic("PositionMeters").subscribe(new float[]{0.0f, 0.0f, 0.0f});
    private final FloatArraySubscriber rotationRadians = t265NetworkTable.getFloatArrayTopic("RotationRadians").subscribe(new float[]{0.0f, 0.0f, 0.0f});

    private Pose2d robotToT265Difference = new Pose2d(0, 0, new Rotation2d(0));
    private double latestResultTimestampSeconds = 0;

    public void updatePeriodically() {
        updateLatestResultTimestampSeconds();
        logInputs();
    }

    public void resetT265Offset(Pose2d robotPose) {
        robotToT265Difference = transform2dToPose2d(getT265Pose().minus(robotPose));
    }

    public Pose2d getEstimatedRobotPose() {
        return transform2dToPose2d(new Transform2d(robotToT265Difference, getT265Pose()));
    }

    public double getLatestResultTimestampSeconds() {
        return latestResultTimestampSeconds;
    }

    private void updateLatestResultTimestampSeconds() {
        if (positionMeters.getLastChange() > latestResultTimestampSeconds)
            latestResultTimestampSeconds = positionMeters.getLastChange();
    }

    private void logInputs() {
        Logger.recordOutput("T265/FPS", framesPerSecond.get());
        Logger.recordOutput("T265/Battery", batteryPercentage.get());
        Logger.recordOutput("T265/RobotPose", getEstimatedRobotPose());
        Logger.recordOutput("T265/T265Pose", getT265Pose());
    }

    private Pose2d transform2dToPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    private Pose2d getT265Pose() {
        return new Pose2d(getT265Translation(), getT265Heading());
    }

    private Translation2d getT265Translation() {
        return new Translation2d(positionMeters.get()[2], -positionMeters.get()[0]);
    }

    private Rotation2d getT265Heading() {
        final double currentYawRadians = rotationRadians.get()[2];
        return Rotation2d.fromRadians(currentYawRadians);
    }
}
