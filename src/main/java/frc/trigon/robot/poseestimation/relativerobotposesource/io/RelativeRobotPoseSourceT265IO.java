package frc.trigon.robot.poseestimation.relativerobotposesource.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceIO;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceInputsAutoLogged;
import org.trigon.utilities.JsonHandler;

public class RelativeRobotPoseSourceT265IO extends RelativeRobotPoseSourceIO {
    private final String hostname;
    private final NetworkTableEntry jsonDumpEntry;

    public RelativeRobotPoseSourceT265IO(String hostname) {
        this.hostname = hostname;
        jsonDumpEntry = NetworkTableInstance.getDefault().getTable(hostname).getEntry("JsonDump");
    }

    @Override
    protected void updateInputs(RelativeRobotPoseSourceInputsAutoLogged inputs) {
        final T265JsonDump jsonDump = readJsonDump();
        if (jsonDump == null) {
            inputs.hasResult = false;
            return;
        }

        inputs.resultTimestampSeconds = jsonDumpEntry.getLastChange();
        inputs.hasResult = true;
        inputs.framesPerSecond = jsonDump.framesPerSecond;
        inputs.batteryPercentage = jsonDump.BatteryPercentage;
        inputs.pose = extractPose(jsonDump);
    }

    private T265JsonDump readJsonDump() {
        return JsonHandler.parseJsonStringToObject(jsonDumpEntry.getString(""), T265JsonDump.class);
    }

    private Pose2d extractPose(T265JsonDump jsonDump) {
        final Translation2d translation = extractTranslation(jsonDump);
        final Rotation2d heading = extractHeading(jsonDump);
        return new Pose2d(translation, heading);
    }

    private Translation2d extractTranslation(T265JsonDump jsonDump) {
        return new Translation2d(jsonDump.xPositionMeters, jsonDump.yPositionMeters);
    }

    private Rotation2d extractHeading(T265JsonDump jsonDump) {
        return Rotation2d.fromRadians(jsonDump.rotationRadians);
    }

    private static class T265JsonDump {
        private int framesPerSecond = 0;
        private double BatteryPercentage = 0;
        private double xPositionMeters = 0;
        private double yPositionMeters = 0;
        private double rotationRadians = 0;
    }
}