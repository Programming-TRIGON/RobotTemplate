package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;

public class ObjectDetectionCamera extends SubsystemBase {
    private final ObjectDetectionCameraInputsAutoLogged objectDetectionCameraInputs = new ObjectDetectionCameraInputsAutoLogged();
    private final ObjectDetectionCameraIO objectDetectionCameraIO;
    private final String hostname;
    private double lastVisibleObjectYawDegrees = 0;
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
    public double getBestObjectYawDegrees() {
        return objectDetectionCameraInputs.visibleObjectsYaw[0];
    }

    public Rotation2d getTrackedObjectYaw() {
        return trackedObjectYaw;
    }

    public void startTrackingBestObject() {
        lastVisibleObjectYawDegrees = getBestObjectYawDegrees();
    }

    private Rotation2d calculateTrackedObjectYaw() {
        double closestTargetToLastVisibleTargetYawDifference = Double.POSITIVE_INFINITY;
        double closestTargetYaw = Double.POSITIVE_INFINITY;
        for (double currentObjectYaw : objectDetectionCameraInputs.visibleObjectsYaw) {
            final double currentObjectToLastVisibleTargetYawDifference = Math.abs(currentObjectYaw - lastVisibleObjectYawDegrees);
            if (currentObjectToLastVisibleTargetYawDifference < closestTargetToLastVisibleTargetYawDifference) {
                closestTargetToLastVisibleTargetYawDifference = currentObjectToLastVisibleTargetYawDifference;
                closestTargetYaw = currentObjectYaw;
            }
        }
        if (closestTargetToLastVisibleTargetYawDifference != Double.POSITIVE_INFINITY)
            lastVisibleObjectYawDegrees = closestTargetYaw;
        
        return Rotation2d.fromDegrees(lastVisibleObjectYawDegrees);
    }

    private ObjectDetectionCameraIO generateIO(String hostname) {
        if (RobotHardwareStats.isReplay())
            return new ObjectDetectionCameraIO();
        if (RobotHardwareStats.isSimulation())
            return new SimulationObjectDetectionCameraIO(hostname);
        return new PhotonObjectDetectionCameraIO(hostname);
    }
}