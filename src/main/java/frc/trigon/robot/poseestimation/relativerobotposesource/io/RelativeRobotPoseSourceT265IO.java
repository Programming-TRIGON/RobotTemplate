package frc.trigon.robot.poseestimation.relativerobotposesource.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceIO;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceInputsAutoLogged;

public class RelativeRobotPoseSourceT265IO extends RelativeRobotPoseSourceIO {
    private final IntegerSubscriber framesPerSecond;
    private final DoubleSubscriber batteryPercentage;
    private final FloatArraySubscriber positionMeters;
    private final FloatArraySubscriber rotationRadians;

    public RelativeRobotPoseSourceT265IO(String hostname) {
        final NetworkTable t265NetworkTable = NetworkTableInstance.getDefault().getTable(hostname);

        framesPerSecond = t265NetworkTable.getIntegerTopic("FPS").subscribe(0);
        batteryPercentage = t265NetworkTable.getDoubleTopic("BatteryPercentage").subscribe(0.0f);
        positionMeters = t265NetworkTable.getFloatArrayTopic("PositionMeters").subscribe(new float[]{0.0f, 0.0f, 0.0f});
        rotationRadians = t265NetworkTable.getFloatArrayTopic("RotationRadians").subscribe(new float[]{0.0f, 0.0f, 0.0f});
    }

    @Override
    protected void updateInputs(RelativeRobotPoseSourceInputsAutoLogged inputs) {
        inputs.framesPerSecond = (int) framesPerSecond.get();
        inputs.batteryPercentage = batteryPercentage.get();
        inputs.pose = getT265Pose();
        inputs.hasNewResult = positionMeters.getLastChange() > inputs.lastResultTimestamp;
        inputs.lastResultTimestamp = positionMeters.getLastChange();
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