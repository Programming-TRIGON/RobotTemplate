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
import java.util.Set;

public class ObjectPoseEstimator extends SubsystemBase {
    private final double deletionThresholdSeconds;
    private final SimulatedGamePieceConstants.GamePieceType gamePieceType;
    private final ObjectDetectionCamera camera;
    /**
     * Holds the position of each detected object and when it was detected.
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
     * Removes objects that have not been detected for {@link ObjectPoseEstimator#deletionThresholdSeconds}.
     */
    @Override
    public void periodic() {
        updateObjectPositions();
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
        final Translation2d closestObjectToIntake = getClosestKnownObjectToPosition(robotPose.transformBy(intakeTransform).getTranslation());
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

    public void removeClosestObjectToPosition(Translation2d position) {
        final Translation2d closestObject = getClosestKnownObjectToPosition(position);
        if (closestObject == null)
            return;
        removeObject(closestObject);
    }

    /**
     * Removes a specific object from the list of known objects in the pose estimator.
     *
     * @param objectPosition the position of the object to be removed. Must be the precise position as stored in the pose estimator.
     */
    public void removeObject(Translation2d objectPosition) {
        objectPositionsToTimestamp.remove(objectPosition);
    }

    /**
     * determines whether any objects are stored in the poseEstimator.
     *
     * @return if there are objects stored in the poseEstimator
     */
    public boolean hasObjects() {
        return !objectPositionsToTimestamp.isEmpty();
    }

    /**
     * Gets the position of the closest object on the field from the 3D rotation of the object relative to the camera.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @return the best object's 2D position on the field (z is assumed to be 0)
     */
    public Translation2d getClosestObjectToRobot() {
        return getClosestKnownObjectToPosition(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation());
    }

    private void updateObjectPositions() {
        HashMap<Translation2d, Translation2d> trackedObjectsToUpdatedPositions = new HashMap<>();

        for (Translation2d visibleObject : camera.getObjectsPositionsOnField(gamePieceType))
            updateObjectPosition(visibleObject, trackedObjectsToUpdatedPositions);

        mergeObjectsIntoHashmap(trackedObjectsToUpdatedPositions);
    }

    private void mergeObjectsIntoHashmap(HashMap<Translation2d, Translation2d> currentToNewObjectPositions) {
        final double currentTimestamp = Timer.getTimestamp();

        currentToNewObjectPositions.keySet().forEach(objectPositionsToTimestamp::remove);
        currentToNewObjectPositions.values().forEach(object -> objectPositionsToTimestamp.put(object, currentTimestamp));
    }

    private void updateObjectPosition(Translation2d object, HashMap<Translation2d, Translation2d> objectsToUpdatedPositions) {
        final Translation2d closestObjectToTargetObject = getClosestKnownObjectToPosition(object);

        if (isObjectNew(object))
            objectsToUpdatedPositions.put(object, object);
        else
            updateHashMapObject(object, closestObjectToTargetObject, objectsToUpdatedPositions);
    }

    private void updateHashMapObject(Translation2d objectUpdate, Translation2d objectToUpdate, HashMap<Translation2d, Translation2d> objectsToUpdatedPositions) {
        if (objectsToUpdatedPositions.containsKey(objectToUpdate)) {
            final Translation2d existingObjectUpdate = objectsToUpdatedPositions.get(objectToUpdate);
            if (shouldReplaceExistingUpdateWithNewUpdate(objectUpdate, existingObjectUpdate, objectToUpdate)) {
                objectsToUpdatedPositions.replace(objectToUpdate, objectUpdate);
                objectsToUpdatedPositions.put(existingObjectUpdate, existingObjectUpdate);
            }
        } else
            objectsToUpdatedPositions.put(objectToUpdate, objectUpdate);
    }

    private boolean shouldReplaceExistingUpdateWithNewUpdate(Translation2d newUpdate, Translation2d existingUpdate, Translation2d objectToUpdate) {
        return newUpdate.getDistance(objectToUpdate) <
                existingUpdate.getDistance(objectToUpdate);
    }

    private boolean isObjectNew(Translation2d object) {
        if (objectPositionsToTimestamp.isEmpty())
            return true;
        return object.getDistance(getClosestKnownObjectToPosition(object)) > ObjectDetectionConstants.TRACKED_OBJECT_TOLERANCE_METERS;
    }

    private Translation2d getClosestKnownObjectToPosition(Translation2d position) {
        return getClosestObjectFromSetToPosition(position, objectPositionsToTimestamp.keySet());
    }

    private Translation2d getClosestObjectFromSetToPosition(Translation2d position, Set<Translation2d> objects) {
        if (objects.isEmpty())
            return null;
        final Translation2d[] objectsTranslations = objects.toArray(Translation2d[]::new);
        Translation2d closestObjectTranslation = objectsTranslations[0];
        double closestObjectDistance = position.getDistance(closestObjectTranslation);

        for (int i = 1; i < objectsTranslations.length; i++) {
            final Translation2d currentObjectTranslation = objectsTranslations[i];
            final double currentObjectDistance = position.getDistance(currentObjectTranslation);
            if (currentObjectDistance < closestObjectDistance) {
                closestObjectDistance = currentObjectDistance;
                closestObjectTranslation = currentObjectTranslation;
            }
        }
        return closestObjectTranslation;
    }

    private void removeOldObjects() {
        objectPositionsToTimestamp.entrySet().removeIf(entry -> isTooOld(entry.getValue()));
    }

    private boolean isTooOld(double timestamp) {
        return Timer.getTimestamp() - timestamp > deletionThresholdSeconds;
    }
}