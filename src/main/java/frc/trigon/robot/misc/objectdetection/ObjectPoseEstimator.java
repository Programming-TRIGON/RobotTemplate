package frc.trigon.robot.misc.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.objectdetection.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

public class ObjectPoseEstimator extends SubsystemBase {
    private final double deletionThresholdSeconds;
    private final SimulatedGamePieceConstants.GamePieceType gamePieceType;
    private final ObjectDetectionCamera camera;
    /**
     * Stores the position of each detected object along with the timestamp of when it was detected.
     */
    private final HashMap<Translation2d, Double> objectPositionsToDetectionTimestamp;

    /**
     * Constructs an ObjectPoseEstimator for estimating the positions of objects detected by camera.
     *
     * @param deletionThresholdSeconds the time in seconds after which an object is considered old and removed
     * @param gamePieceType            the type of game piece to track
     * @param camera                   the camera used for detecting objects
     */
    public ObjectPoseEstimator(double deletionThresholdSeconds,
                               SimulatedGamePieceConstants.GamePieceType gamePieceType,
                               ObjectDetectionCamera camera) {
        this.deletionThresholdSeconds = deletionThresholdSeconds;
        this.gamePieceType = gamePieceType;
        this.camera = camera;
        this.objectPositionsToDetectionTimestamp = new HashMap<>();
    }

    /**
     * Updates the object positions based on the camera detected objects.
     * Removes objects that have not been detected for a certain time frame, defined in {@link ObjectPoseEstimator#deletionThresholdSeconds}.
     */
    @Override
    public void periodic() {
        updateTrackedObjectsPositions();
        removeOldObjects();
        Logger.recordOutput("ObjectPoseEstimator/knownObjectPositions", getObjectsOnField().toArray(Translation2d[]::new));
    }

    /**
     * Gets the position of all known objects on the field.
     *
     * @return a list of Translation2d representing the positions of objects on the field
     */
    public ArrayList<Translation2d> getObjectsOnField() {
        return new ArrayList<>(objectPositionsToDetectionTimestamp.keySet());
    }

    /**
     * Removes the closest object to the robot from the list of objects in the pose estimator.
     */
    public void removeClosestObjectToRobot() {
        final Translation2d closestObject = getClosestObjectToRobot();
        if (closestObject == null)
            return;
        removeObject(closestObject);
    }

    /**
     * Removes the closest object to the intake from the list of objects in the pose estimator.
     *
     * @param intakeTransform the transform of the intake relative to the robot
     */
    public void removeClosestObjectToIntake(Transform2d intakeTransform) {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d closestObjectToIntake = getClosestTrackedObjectToPosition(robotPose.transformBy(intakeTransform).getTranslation());
        if (closestObjectToIntake == null)
            return;
        removeObject(closestObjectToIntake);
    }

    /**
     * Removes the closest object to a given pose from the list of objects in the pose estimator.
     *
     * @param fieldRelativePose the pose to which the removed object is closest
     */
    public void removeClosestObjectToPose(Pose2d fieldRelativePose) {
        removeClosestObjectToPosition(fieldRelativePose.getTranslation());
    }

    /**
     * Removes the closest object to a given position from the stored objects in the pose estimator.
     *
     * @param position the position to which the removed object is closest
     */
    public void removeClosestObjectToPosition(Translation2d position) {
        final Translation2d closestObject = getClosestTrackedObjectToPosition(position);
        if (closestObject == null)
            return;
        removeObject(closestObject);
    }

    /**
     * Removes a specific object from the stored objects in the pose estimator.
     * Unlike {@link #removeClosestObjectToPosition} which removes the closest object to a given position.
     *
     * @param objectPosition the position of the object to be removed. Must be the precise position as stored in the pose estimator.
     */
    public void removeObject(Translation2d objectPosition) {
        objectPositionsToDetectionTimestamp.remove(objectPosition);
    }

    /**
     * Returns whether any objects are stored in the pose estimator.
     *
     * @return if there are objects stored in the pose estimator
     */
    public boolean hasObjects() {
        return !objectPositionsToDetectionTimestamp.isEmpty();
    }

    /**
     * Gets the position of the closest object to the robot.
     *
     * @return the closest object to the robot
     */
    public Translation2d getClosestObjectToRobot() {
        return getClosestTrackedObjectToPosition(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation());
    }

    private void updateTrackedObjectsPositions() {
        final Translation2d[] visibleObjects = camera.getObjectPositionsOnField(gamePieceType);
        final HashMap<Translation2d, Translation2d> trackedObjectsToUpdatedPositions = new HashMap<>();

        for (Translation2d visibleObject : visibleObjects)
            updateObjectPosition(visibleObject, trackedObjectsToUpdatedPositions);

        applyObjectUpdates(trackedObjectsToUpdatedPositions);
    }

    private void applyObjectUpdates(HashMap<Translation2d, Translation2d> currentToNewObjectPositions) {
        final double currentTimestamp = Timer.getTimestamp();

        objectPositionsToDetectionTimestamp.keySet().removeAll(currentToNewObjectPositions.keySet());
        currentToNewObjectPositions.values().forEach(object -> objectPositionsToDetectionTimestamp.put(object, currentTimestamp));
    }

    private void updateObjectPosition(Translation2d objectUpdate, HashMap<Translation2d, Translation2d> trackedObjectsToUpdatedPositions) {
        final Translation2d closestAvailableTrackedObjectToVisibleObject = getClosestAvailableObjectToUpdate(objectUpdate, trackedObjectsToUpdatedPositions);
        if (closestAvailableTrackedObjectToVisibleObject == null) {
            trackedObjectsToUpdatedPositions.put(objectUpdate, objectUpdate);
            return;
        }
        final Translation2d previousUpdate = trackedObjectsToUpdatedPositions.get(closestAvailableTrackedObjectToVisibleObject);
        trackedObjectsToUpdatedPositions.put(closestAvailableTrackedObjectToVisibleObject, objectUpdate);
        if (previousUpdate != null)
            updateObjectPosition(previousUpdate, trackedObjectsToUpdatedPositions);
    }

    private Translation2d getClosestAvailableObjectToUpdate(Translation2d update, HashMap<Translation2d, Translation2d> objectsWithUpdates) {
        final Set<Translation2d> availableObjectsToUpdate = getAvailableObjectsToUpdate(update, objectsWithUpdates);
        if (availableObjectsToUpdate == null || availableObjectsToUpdate.isEmpty())
            return null;
        return getClosestObjectFromSetToPosition(update, availableObjectsToUpdate);
    }

    private Set<Translation2d> getAvailableObjectsToUpdate(Translation2d update, HashMap<Translation2d, Translation2d> objectsWithUpdates) {
        if (objectPositionsToDetectionTimestamp.isEmpty())
            return null;
        final Set<Translation2d> availableObjects = new HashSet<>();
        for (Translation2d currentObject : objectPositionsToDetectionTimestamp.keySet()) {
            final double updateDistanceFromCurrentObject = update.getDistance(currentObject);
            if (updateDistanceFromCurrentObject > ObjectDetectionConstants.TRACKED_OBJECT_TOLERANCE_METERS)
                continue;
            if (!objectsWithUpdates.containsKey(currentObject)) {
                availableObjects.add(currentObject);
                continue;
            }
            if (isNewUpdateCloserThanPreviousUpdate(update, objectsWithUpdates.get(currentObject), currentObject))
                availableObjects.add(currentObject);
        }
        return availableObjects;
    }

    private boolean isNewUpdateCloserThanPreviousUpdate(Translation2d newUpdate, Translation2d previousUpdate, Translation2d object) {
        return newUpdate.getDistance(object) < previousUpdate.getDistance(object);
    }

    private Translation2d getClosestTrackedObjectToPosition(Translation2d position) {
        return getClosestObjectFromSetToPosition(position, objectPositionsToDetectionTimestamp.keySet());
    }

    private Translation2d getClosestObjectFromSetToPosition(Translation2d position, Set<Translation2d> objects) {
        if (objects.isEmpty())
            return null;
        Translation2d closestObjectTranslation = null;
        double closestObjectDistance = Double.MAX_VALUE;

        for (Translation2d object : objects) {
            final double currentObjectDistance = position.getDistance(object);
            if (currentObjectDistance < closestObjectDistance) {
                closestObjectDistance = currentObjectDistance;
                closestObjectTranslation = object;
            }
        }
        return closestObjectTranslation;
    }

    private void removeOldObjects() {
        objectPositionsToDetectionTimestamp.entrySet().removeIf(entry -> hasObjectExpired(entry.getValue()));
    }

    private boolean hasObjectExpired(double timestamp) {
        return Timer.getTimestamp() - timestamp > deletionThresholdSeconds;
    }
}