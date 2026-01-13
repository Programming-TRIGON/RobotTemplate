package frc.trigon.robot.misc.objectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.objectDetection.objectdetectioncamera.ObjectDetectionCamera;
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
    private final HashMap<Translation2d, Double> objectPositionsToTimestamp;

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
        this.objectPositionsToTimestamp = new HashMap<>();
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
        return new ArrayList<>(objectPositionsToTimestamp.keySet());
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
        objectPositionsToTimestamp.remove(objectPosition);
    }

    /**
     * Returns whether any objects are stored in the pose estimator.
     *
     * @return if there are objects stored in the pose estimator
     */
    public boolean hasObjects() {
        return !objectPositionsToTimestamp.isEmpty();
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

        for (Translation2d visibleObject : visibleObjects) {
            updateObjectPosition(visibleObject, trackedObjectsToUpdatedPositions);
        }

        updateObjectUpdates(trackedObjectsToUpdatedPositions);
    }

    private void updateObjectUpdates(HashMap<Translation2d, Translation2d> currentToNewObjectPositions) {
        final double currentTimestamp = Timer.getTimestamp();

        objectPositionsToTimestamp.keySet().removeAll(currentToNewObjectPositions.keySet());
        currentToNewObjectPositions.values().forEach(object -> objectPositionsToTimestamp.put(object, currentTimestamp));
    }

    private void updateObjectPosition(Translation2d object, HashMap<Translation2d, Translation2d> trackedObjectsToUpdatedPositions) {
        final Translation2d nearestTrackedObject = getClosestTrackedObjectToPosition(object);
        if (isObjectNotStored(object, nearestTrackedObject))
            trackedObjectsToUpdatedPositions.put(object, object);
        else
            assignUpdateToTrackedObject(object, nearestTrackedObject, trackedObjectsToUpdatedPositions);
    }

    private void assignUpdateToTrackedObject(Translation2d objectsUpdatedPosition, Translation2d objectToUpdate, HashMap<Translation2d, Translation2d> trackedObjectsToUpdatedPositions) {
        if (trackedObjectsToUpdatedPositions.containsKey(objectToUpdate)) {
            final Set<Translation2d> excludedKnownObjects = new HashSet<>();
            addClosestUpdatedPositionToHashMap(objectsUpdatedPosition, objectToUpdate, trackedObjectsToUpdatedPositions, excludedKnownObjects);
        } else
            trackedObjectsToUpdatedPositions.put(objectToUpdate, objectsUpdatedPosition);
    }

    private void addClosestUpdatedPositionToHashMap(Translation2d objectsUpdatedPosition, Translation2d objectToUpdate, HashMap<Translation2d, Translation2d> trackedObjectsToUpdatedPositions, Set<Translation2d> excludedKnownObjects) {
        final Translation2d existingUpdatedPosition = trackedObjectsToUpdatedPositions.get(objectToUpdate);
        if (isNewObjectUpdateCloserThanPreviousUpdate(objectsUpdatedPosition, existingUpdatedPosition, objectToUpdate)) {
            trackedObjectsToUpdatedPositions.replace(objectToUpdate, objectsUpdatedPosition);
            reassignDiscardedObjectUpdate(existingUpdatedPosition, objectToUpdate, trackedObjectsToUpdatedPositions, excludedKnownObjects);
            return;
        }
        reassignDiscardedObjectUpdate(objectsUpdatedPosition, objectToUpdate, trackedObjectsToUpdatedPositions, excludedKnownObjects);
    }

    private void reassignDiscardedObjectUpdate(Translation2d discardedObjectUpdate, Translation2d previousClosestTrackedObject, HashMap<Translation2d, Translation2d> trackedObjectsToUpdatedPositions, Set<Translation2d> excludedKnownObjects) {
        excludedKnownObjects.add(previousClosestTrackedObject);
        final Translation2d nextClosestObjectToDiscardedObjectUpdate = getNextClosestKnownObjectToPosition(discardedObjectUpdate, excludedKnownObjects);
        if (nextClosestObjectToDiscardedObjectUpdate == null ||
                discardedObjectUpdate.getDistance(nextClosestObjectToDiscardedObjectUpdate) > ObjectDetectionConstants.TRACKED_OBJECT_TOLERANCE_METERS) {
            trackedObjectsToUpdatedPositions.put(discardedObjectUpdate, discardedObjectUpdate);
            return;
        }
        addClosestUpdatedPositionToHashMap(discardedObjectUpdate, nextClosestObjectToDiscardedObjectUpdate, trackedObjectsToUpdatedPositions, excludedKnownObjects);
    }

    private boolean isNewObjectUpdateCloserThanPreviousUpdate(Translation2d newUpdate, Translation2d existingUpdate, Translation2d trackedObject) {
        return newUpdate.getDistance(trackedObject) <
                existingUpdate.getDistance(trackedObject);
    }

    private boolean isObjectNotStored(Translation2d object, Translation2d nearestTrackedObject) {
        if (nearestTrackedObject == null)
            return true;
        return object.getDistance(nearestTrackedObject) > ObjectDetectionConstants.TRACKED_OBJECT_TOLERANCE_METERS;
    }

    private Translation2d getNextClosestKnownObjectToPosition(Translation2d position, Set<Translation2d> excludedKnownObjects) {
        Set<Translation2d> candidateObjects = new HashSet<>(objectPositionsToTimestamp.keySet());
        candidateObjects.removeAll(excludedKnownObjects);
        return getClosestObjectFromSetToPosition(position, candidateObjects);
    }

    private Translation2d getClosestTrackedObjectToPosition(Translation2d position) {
        return getClosestObjectFromSetToPosition(position, objectPositionsToTimestamp.keySet());
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
        objectPositionsToTimestamp.entrySet().removeIf(entry -> hasObjectExpired(entry.getValue()));
    }

    private boolean hasObjectExpired(double timestamp) {
        return Timer.getTimestamp() - timestamp > deletionThresholdSeconds;
    }
}