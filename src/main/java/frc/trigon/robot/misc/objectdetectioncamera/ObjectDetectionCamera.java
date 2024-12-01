package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;

/**
 * An object detection camera is a class that represents a camera that detects objects other than apriltags, most likely game pieces.
 */
public class ObjectDetectionCamera extends SubsystemBase {
    private final ObjectDetectionCameraInputsAutoLogged objectDetectionCameraInputs = new ObjectDetectionCameraInputsAutoLogged();
    private final ObjectDetectionCameraIO objectDetectionCameraIO;
    private final String hostname;
    private Rotation2d trackedObjectYaw = new Rotation2d();
    private boolean wasVisible = false;

    public ObjectDetectionCamera(String hostname) {
        this.hostname = hostname;
        objectDetectionCameraIO = generateIO(hostname);
    }

    @Override
    public void periodic() {
        objectDetectionCameraIO.updateInputs(objectDetectionCameraInputs);
        Logger.processInputs(hostname, objectDetectionCameraInputs);
    }

    /**
     * Starts the tracking of the best visible target and remains tracking that target until it is no longer visible.
     * Tracking an object is locking on to one target and allows you to stay locked on to one target.
     * This is used when there is more than one target that might change as the robot moves to stabilize the result and ensure that it is following the same target that it started with.
     * This needs to be called each time you want to track a new object.
     */
    public void trackObject() {
        if (hasTargets() && !wasVisible) {
            wasVisible = true;
            startTrackingBestObject();
            trackedObjectYaw = calculateTrackedObjectYaw();
            return;
        }
        if (!hasTargets()) {
            wasVisible = false;
            return;
        }
        trackedObjectYaw = calculateTrackedObjectYaw();
    }

    public boolean hasTargets() {
        return objectDetectionCameraInputs.hasTargets;
    }

    /**
     * @return the yaw (x-axis position) of the target object
     */
    public Rotation2d getBestObjectYaw() {
        return objectDetectionCameraInputs.visibleObjectsYaw[0];
    }

    /**
     * @return the yaw (x-axis position) of the current tracked object
     */
    public Rotation2d getTrackedObjectYaw() {
        return trackedObjectYaw;
    }

    /**
     * Start tracking of the best visible object.
     * It remains tracking the object until it is no longer visible.
     */
    public void startTrackingBestObject() {
        trackedObjectYaw = getBestObjectYaw();
    }

    /**
     * Calculates the yaw (x-axis position) of the object that the camera is currently tracking.
     *
     * @return the yaw of the tracked object
     */
    private Rotation2d calculateTrackedObjectYaw() {
        double closestTargetToTrackedTargetYawDifference = Double.POSITIVE_INFINITY;
        Rotation2d closestToTrackedTargetYaw = new Rotation2d();

        for (Rotation2d currentObjectYaw : objectDetectionCameraInputs.visibleObjectsYaw) {
            final double currentObjectToTrackedTargetYawDifference = Math.abs(currentObjectYaw.getRadians() - trackedObjectYaw.getRadians());
            if (currentObjectToTrackedTargetYawDifference < closestTargetToTrackedTargetYawDifference) {
                closestTargetToTrackedTargetYawDifference = currentObjectToTrackedTargetYawDifference;
                closestToTrackedTargetYaw = currentObjectYaw;
            }
        }

        return closestToTrackedTargetYaw;
    }

    private ObjectDetectionCameraIO generateIO(String hostname) {
        if (RobotHardwareStats.isReplay())
            return new ObjectDetectionCameraIO();
        if (RobotHardwareStats.isSimulation())
            return new SimulationObjectDetectionCameraIO(hostname);
        return new PhotonObjectDetectionCameraIO(hostname);
    }
}