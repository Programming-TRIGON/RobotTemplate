package frc.trigon.robot.poseestimation.t265;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import org.littletonrobotics.junction.Logger;

public class T265 {
    private static T265 INSTANCE;
    private final NetworkTable t265NetworkTable = NetworkTableInstance.getDefault().getTable("T265");

    private final IntegerSubscriber framesPerSecond = t265NetworkTable.getIntegerTopic("FPS").subscribe(0);
    private final DoubleSubscriber batteryLevel = t265NetworkTable.getDoubleTopic("BatteryLevel").subscribe(0.0f);
    private final FloatArraySubscriber position = t265NetworkTable.getFloatArrayTopic("Position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
    private final FloatArraySubscriber rotation = t265NetworkTable.getFloatArrayTopic("Rotation").subscribe(new float[]{0.0f, 0.0f, 0.0f});

    private Transform2d t265ToRobotTransform = new Transform2d(0, 0, new Rotation2d(0));
    private double latestResultTimestampSeconds = 0;

    public static T265 getInstance() {
        if (INSTANCE == null)
            INSTANCE = new T265();
        return INSTANCE;
    }

    public void updatePeriodically() {
        updateLatestResultTimestampSeconds();
        logInputs();
    }

    public void resetT265Offset(Pose2d robotPose) {
        t265ToRobotTransform = robotPose.minus(getT265Pose());
    }

    public double getLatestResultTimestampSeconds() {
        return latestResultTimestampSeconds;
    }

    public Pose2d getEstimatedRobotPose() {
        final Pose2d t265Pose = getT265Pose();
        return new Pose2d(
                t265Pose.getTranslation().minus(t265ToRobotTransform.getTranslation()),
                t265Pose.getRotation().minus(t265ToRobotTransform.getRotation())
        );
    }

    private Pose2d getT265Pose() {
        return new Pose2d(getT265Translation(), getT265Heading());
    }

    private Translation2d getT265Translation() {
        return new Translation2d(position.get()[2], -position.get()[0]);
    }

    private Rotation2d getT265Heading() {
        final double currentYawRadians = rotation.get()[2];
        return Rotation2d.fromRadians(currentYawRadians);
    }

    private void updateLatestResultTimestampSeconds() {
        if (position.getLastChange() > latestResultTimestampSeconds)
            latestResultTimestampSeconds = position.getLastChange();
    }

    private void logInputs() {
        Logger.recordOutput("T265/FPS", framesPerSecond.get());
        Logger.recordOutput("T265/Battery", batteryLevel.get());
        Logger.recordOutput("T265/RobotPose", getEstimatedRobotPose());
        Logger.recordOutput("T265/T265Pose", getT265Pose());
    }
}
