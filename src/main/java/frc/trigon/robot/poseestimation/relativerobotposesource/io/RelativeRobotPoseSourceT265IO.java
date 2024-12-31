package frc.trigon.robot.poseestimation.relativerobotposesource.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceIO;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceInputsAutoLogged;
import org.trigon.utilities.JsonHandler;

public class RelativeRobotPoseSourceT265IO extends RelativeRobotPoseSourceIO {
    private JsonDump jsonDump = new JsonDump();
    private final String hostname;

    public RelativeRobotPoseSourceT265IO(String hostname) {
        this.hostname = hostname;
    }

    @Override
    protected void updateInputs(RelativeRobotPoseSourceInputsAutoLogged inputs) {
        updateJsonDump();
        final Pose2d lastPose = inputs.pose;

        inputs.framesPerSecond = jsonDump.framesPerSecond;
        inputs.batteryPercentage = jsonDump.BatteryPercentage;
        inputs.pose = getT265Pose();
        inputs.lastResultTimestamp = lastPose == inputs.pose ? inputs.lastResultTimestamp : Timer.getTimestamp();
    }

    private void updateJsonDump() {
        final String jsonString = NetworkTableInstance.getDefault().getEntry(hostname).getString(null);
        if (jsonString != null)
            jsonDump = JsonHandler.parseJsonStringToObject(jsonString, JsonDump.class);
    }

    private Pose2d getT265Pose() {
        return new Pose2d(getT265Translation(), getT265Heading());
    }

    private Translation2d getT265Translation() {
        return new Translation2d(jsonDump.xPositionMeters, jsonDump.yPositionMeters);
    }

    private Rotation2d getT265Heading() {
        return Rotation2d.fromRadians(jsonDump.rotationRadians);
    }

    private static class JsonDump {
        private int framesPerSecond = 0;
        private double BatteryPercentage = 0;
        private double xPositionMeters = 0;
        private double yPositionMeters = 0;
        private double rotationRadians = 0;
    }
}