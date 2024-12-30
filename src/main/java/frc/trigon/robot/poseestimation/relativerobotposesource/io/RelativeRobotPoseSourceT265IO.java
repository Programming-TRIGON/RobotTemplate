package frc.trigon.robot.poseestimation.relativerobotposesource.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceIO;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceInputsAutoLogged;
import org.trigon.utilities.JsonHandler;

public class RelativeRobotPoseSourceT265IO extends RelativeRobotPoseSourceIO {
    private final String fileName;
    private JsonDump jsonDump;

    public RelativeRobotPoseSourceT265IO(String hostname) {
        fileName = hostname + ".json";
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
        jsonDump = JsonHandler.parseJsonFileToObject(fileName, JsonDump.class);
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
}

class JsonDump {
    protected int framesPerSecond = 0;
    protected double BatteryPercentage = 0;
    protected double xPositionMeters = 0;
    protected double yPositionMeters = 0;
    protected double rotationRadians = 0;
}