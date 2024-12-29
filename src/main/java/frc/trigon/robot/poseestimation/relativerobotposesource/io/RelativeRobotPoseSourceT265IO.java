package frc.trigon.robot.poseestimation.relativerobotposesource.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceIO;
import frc.trigon.robot.poseestimation.relativerobotposesource.RelativeRobotPoseSourceInputsAutoLogged;
import org.trigon.utilities.JsonHandler;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RelativeRobotPoseSourceT265IO extends RelativeRobotPoseSourceIO {
    private final Supplier<Integer> framesPerSecond;
    private final DoubleSupplier batteryPercentage;
    private final DoubleSupplier xPositionMeters, yPositionMeters, rotationRadians;

    private Pose2d lastPose = new Pose2d();

    public RelativeRobotPoseSourceT265IO(String hostname) {
        framesPerSecond = () -> JsonHandler.parseJsonStringToObject("FPS", Integer.TYPE);
        batteryPercentage = () -> JsonHandler.parseJsonStringToObject("BatteryPercentage", Double.TYPE);
        xPositionMeters = () -> JsonHandler.parseJsonStringToObject("XPositionMeters", Double.TYPE);
        yPositionMeters = () -> JsonHandler.parseJsonStringToObject("YPositionMeters", Double.TYPE);
        rotationRadians = () -> JsonHandler.parseJsonStringToObject("RotationRadians", Double.TYPE);
    }

    @Override
    protected void updateInputs(RelativeRobotPoseSourceInputsAutoLogged inputs) {
        lastPose = inputs.pose;

        inputs.framesPerSecond = framesPerSecond.get();
        inputs.batteryPercentage = batteryPercentage.getAsDouble();
        inputs.pose = getT265Pose();
        inputs.lastResultTimestamp = lastPose == inputs.pose ? inputs.lastResultTimestamp : Timer.getTimestamp();
    }

    private Pose2d getT265Pose() {
        return new Pose2d(getT265Translation(), getT265Heading());
    }

    private Translation2d getT265Translation() {
        return new Translation2d(xPositionMeters.getAsDouble(), yPositionMeters.getAsDouble());
    }

    private Rotation2d getT265Heading() {
        return Rotation2d.fromRadians(rotationRadians.getAsDouble());
    }
}