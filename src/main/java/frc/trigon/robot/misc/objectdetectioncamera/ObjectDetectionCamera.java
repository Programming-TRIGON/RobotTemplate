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
    private Rotation2d lastTrackedTargetYaw = new Rotation2d();
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
        lastTrackedTargetYaw = getBestObjectYaw();
    }

    /**
     * Calculates the yaw (x-axis position) of the object that it is currently tracking.
     *
     * @return the yaw of the tracked object
     */
    private Rotation2d calculateTrackedObjectYaw() {
        double closestTargetToLastVisibleTargetYawDifference = Double.POSITIVE_INFINITY;
        Rotation2d closestToTrackedTargetYaw = new Rotation2d();
        for (Rotation2d currentObjectYaw : objectDetectionCameraInputs.visibleObjectsYaw) {
            final double currentObjectToLastVisibleTargetYawDifference = Math.abs(currentObjectYaw.getRadians() - lastTrackedTargetYaw.getRadians());
            if (currentObjectToLastVisibleTargetYawDifference < closestTargetToLastVisibleTargetYawDifference) {
                closestTargetToLastVisibleTargetYawDifference = currentObjectToLastVisibleTargetYawDifference;
                closestToTrackedTargetYaw = currentObjectYaw;
            }
        }
        if (closestTargetToLastVisibleTargetYawDifference != Double.POSITIVE_INFINITY)
            lastTrackedTargetYaw = closestToTrackedTargetYaw;

        return lastTrackedTargetYaw;
    }

    private ObjectDetectionCameraIO generateIO(String hostname) {
        if (RobotHardwareStats.isReplay())
            return new ObjectDetectionCameraIO();
        if (RobotHardwareStats.isSimulation())
            return new SimulationObjectDetectionCameraIO(hostname);
        return new PhotonObjectDetectionCameraIO(hostname);
    }
}