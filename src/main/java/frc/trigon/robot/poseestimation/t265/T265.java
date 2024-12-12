package frc.trigon.robot.poseestimation.t265;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import org.littletonrobotics.junction.Logger;

public class T265 {
    private static T265 INSTANCE;
    private final NetworkTable nt4Table = NetworkTableInstance.getDefault().getTable("T265");

    private final IntegerSubscriber t265FPS = nt4Table.getIntegerTopic("FPS").subscribe(0);
    private final DoubleSubscriber t265Battery = nt4Table.getDoubleTopic("BatteryLevel").subscribe(0.0f);
    private final FloatArraySubscriber t265Position = nt4Table.getFloatArrayTopic("Position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
    private final FloatArraySubscriber t265Rotation = nt4Table.getFloatArrayTopic("Rotation").subscribe(new float[]{0.0f, 0.0f, 0.0f});

    private Translation2d t265ToRobotTranslationOffset = new Translation2d(0, 0);
    private Rotation2d t265YawToRobotYawOffset = new Rotation2d(0);
    private double latestResultTimestamp = 0;

    public static T265 getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new T265();
        }
        return INSTANCE;
    }

    public void updatePeriodically() {
        Logger.recordOutput("FPS", t265FPS.get());
        Logger.recordOutput("Battery", t265Battery.get());
        Logger.recordOutput("RobotPose", getEstimatedRobotPose());
        Logger.recordOutput("T265Pose", getT265Pose());
    }

    public void resetT265Offset(Pose2d robotPose) {
        t265ToRobotTranslationOffset = robotPose.getTranslation().minus(getT265Translation());
        t265YawToRobotYawOffset = robotPose.getRotation().minus(getT265Heading());
    }

    public double getLatestResultTimestampSeconds() {
        if (t265Position.getLastChange() > latestResultTimestamp)
            latestResultTimestamp = t265Position.getLastChange();
        return latestResultTimestamp;
    }

    public Pose2d getEstimatedRobotPose() {
        Translation2d robotTranslation = getT265Translation().minus(t265ToRobotTranslationOffset);
        Rotation2d robotRotation = getT265Heading().minus(t265YawToRobotYawOffset);
        return new Pose2d(robotTranslation, robotRotation);
    }

    private Pose2d getT265Pose() {
        return new Pose2d(getT265Translation(), getT265Heading());
    }

    private Translation2d getT265Translation() {
        return new Translation2d(t265Position.get()[2], -t265Position.get()[0]);
    }

    private Rotation2d getT265Heading() {
        final double currentYawRadians = t265Rotation.get()[2];
        return Rotation2d.fromRadians(currentYawRadians);
    }
}
