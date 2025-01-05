package frc.trigon.robot.poseestimation.relativerobotposesource.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceIO;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceInputsAutoLogged;
import org.trigon.utilities.JsonHandler;

public class RelativeRobotPoseSourceT265IO extends RelativeRobotPoseSourceIO {
    private T265JsonDump t265JsonDump = new T265JsonDump();
    private final String hostname;

    public RelativeRobotPoseSourceT265IO(String hostname) {
        this.hostname = hostname;
    }

    @Override
    protected void updateInputs(RelativeRobotPoseSourceInputsAutoLogged inputs) {
        updateJsonDump();
        final Pose2d lastPose = inputs.pose;

        inputs.framesPerSecond = t265JsonDump.framesPerSecond;
        inputs.batteryPercentage = t265JsonDump.BatteryPercentage;
        inputs.pose = getT265Pose();
        inputs.lastResultTimestamp = lastPose == inputs.pose ? inputs.lastResultTimestamp : NetworkTableInstance.getDefault().getTable(hostname).getValue("").getServerTime();
    }

    private void updateJsonDump() {
        final String jsonString = NetworkTableInstance.getDefault().getTable(hostname).getValue("").getString();
        t265JsonDump = JsonHandler.parseJsonStringToObject(jsonString, T265JsonDump.class);
    }

    private Pose2d getT265Pose() {
        return new Pose2d(getT265Translation(), getT265Heading());
    }

    private Translation2d getT265Translation() {
        return new Translation2d(t265JsonDump.xPositionMeters, t265JsonDump.yPositionMeters);
    }

    private Rotation2d getT265Heading() {
        return Rotation2d.fromRadians(t265JsonDump.rotationRadians);
    }

    private static class T265JsonDump {
        private int framesPerSecond = 0;
        private double BatteryPercentage = 0;
        private double xPositionMeters = 0;
        private double yPositionMeters = 0;
        private double rotationRadians = 0;
    }
}