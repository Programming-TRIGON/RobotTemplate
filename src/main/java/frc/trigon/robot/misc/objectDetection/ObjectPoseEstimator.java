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
import java.util.function.BiFunction;

public class ObjectPoseEstimator extends SubsystemBase {
    private final double deletionThresholdSeconds;
    private final SimulatedGamePieceConstants.GamePieceType gamePieceType;
    private final ObjectDetectionCamera[] cameras;
    private final HashMap<Translation2d, Double> objectPositionsToTimeStamp;

    /**
     * Constructs an ObjectPoseEstimator for estimating the positions of objects detected by camera.
     *
     * @param deletionThresholdSeconds the time in seconds after which an object is considered old and removed
     * @param gamePieceType            the type of game piece to track
     * @param cameras                  the cameras used for detecting objects
     */
    public ObjectPoseEstimator(double deletionThresholdSeconds,
                               SimulatedGamePieceConstants.GamePieceType gamePieceType,
                               ObjectDetectionCamera... cameras) {
        this.deletionThresholdSeconds = deletionThresholdSeconds;
        this.gamePieceType = gamePieceType;
        this.cameras = cameras;
        this.objectPositionsToTimeStamp = new HashMap<>();
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
        return new ArrayList<>(objectPositionsToTimeStamp.keySet());
    }

    /**
     * Removes the closest object to the robot from the list of objects in the pose estimator.
     */
    public void removeClosestObjectToRobot() {
        final Translation2d closestObject = getClosestObjectToRobot();
        removeObject(closestObject);
    }

    /**
     * Removes the closest object to the intake from the list of objects in the pose estimator.
     *
     * @param intakeTransform the transform of the intake relative to the robot
     */
    public void removeClosestObjectToIntake(Transform2d intakeTransform) {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        removeObject(getClosestKnownObjectToPosition(robotPose.transformBy(intakeTransform).getTranslation()));
    }

    /**
     * Removes the closest object to a given pose from the list of objects in the pose estimator.
     *
     * @param fieldRelativePose the pose to which the removed object is closest
     */
    public void removeClosestObjectToPose(Pose2d fieldRelativePose) {
        final Translation2d closestObject = getClosestKnownObjectToPosition(fieldRelativePose.getTranslation());
        removeObject(closestObject);
    }

    public void removeClosestObjectToPosition(Translation2d position) {
        final Translation2d closestObject = getClosestKnownObjectToPosition(position);
        removeObject(closestObject);
    }

    /**
     * Removes a specific object from the list of known objects in the pose estimator.
     *
     * @param objectPosition the position of the object to be removed. Must be the precise position as stored in the pose estimator.
     */
    public void removeObject(Translation2d objectPosition) {
        objectPositionsToTimeStamp.remove(objectPosition);
    }

    /**
     * determines whether any objects are stored in the poseEstimator.
     *
     * @return if there are objects stored in the poseEstimator
     */
    public boolean hasObjects() {
        return !getObjectsOnField().isEmpty();
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


    /**
     * Gets the position of the closest object to a given position on the field.
     *
     * @param position the position to which the returned object is closest
     * @return the closest object's position on the field, or null if no objects are known
     */
    public Translation2d getClosestKnownObjectToPosition(Translation2d position) {
        return getClosestObjectFromSetToPosition(position, objectPositionsToTimeStamp.keySet());
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

    /**
     * Updates the positions of previously detected objects, or adds them if they are new.
     *
     * <p>{@code currentToNewObjectPositions} maps previously detected objects to the
     * positions they should be updated to.</p>
     *
     * <p>Each object detected by the camera(s) is compared to the closest previously
     * detected object to determine whether it is new or already tracked. If it is
     * already tracked, it is added to {@code currentToNewObjectPositions} as an update.</p>
     *
     * <p>If multiple objects detected by the camera(s) are closest to the same
     * previously detected object, the object whose position is closest to that
     * previously detected object is kept. The others are discarded.</p>
     *
     * <p>This assumes that such detections represent the same physical object seen
     * by different cameras, with small positional differences.</p>
     */
    private void updateObjectPositions() {
        final double currentTimestamp = Timer.getTimestamp();
        HashMap<Translation2d, Translation2d> currentToNewObjectPositions = new HashMap<>();

        for (ObjectDetectionCamera camera : cameras) {
            for (Translation2d visibleObject : camera.getObjectsPositionsOnField(gamePieceType)) {
                final Translation2d closestObjectToVisibleObject = getClosestKnownObjectToPosition(visibleObject);

                if (isObjectNew(visibleObject) && !isObjectsDistanceWithinTolerance(visibleObject, getClosestObjectFromSetToPosition(visibleObject, currentToNewObjectPositions.keySet()))) {
                    currentToNewObjectPositions.put(visibleObject, visibleObject);
                    continue;
                }
                if (isObjectsDistanceWithinTolerance(visibleObject, closestObjectToVisibleObject))
                    currentToNewObjectPositions.merge(
                            closestObjectToVisibleObject,
                            visibleObject,
                            (oldVisibleObject, currentVisibleObject) ->
                                    currentVisibleObject.getDistance(closestObjectToVisibleObject) < oldVisibleObject.getDistance(closestObjectToVisibleObject)
                                            ? currentVisibleObject
                                            : oldVisibleObject);
            }
        }
        currentToNewObjectPositions.keySet().forEach(objectPositionsToTimeStamp::remove);
        currentToNewObjectPositions.values().forEach(object -> objectPositionsToTimeStamp.put(object, currentTimestamp));
    }

    private BiFunction<? super Translation2d, ? super Translation2d, ? extends Translation2d> getClosestVisibleObjectFunction(Translation2d position) {
        return (oldVisibleObject, currentVisibleObject) ->
                currentVisibleObject.getDistance(position) < oldVisibleObject.getDistance(position)
                        ? currentVisibleObject
                        : oldVisibleObject;
    }

    private boolean isObjectsDistanceWithinTolerance(Translation2d firstObject, Translation2d secondObject) {
        return firstObject.getDistance(secondObject) < ObjectDetectionConstants.TRACKED_OBJECT_TOLERANCE_METERS;
    }

    private boolean isObjectNew(Translation2d object) {
        if (objectPositionsToTimeStamp.isEmpty())
            return true;
        return object.getDistance(getClosestKnownObjectToPosition(object)) > ObjectDetectionConstants.TRACKED_OBJECT_TOLERANCE_METERS;
    }

    private void removeOldObjects() {
        objectPositionsToTimeStamp.entrySet().removeIf(entry -> isTooOld(entry.getValue()));
    }

    private boolean isTooOld(double timestamp) {
        return Timer.getTimestamp() - timestamp > deletionThresholdSeconds;
    }
}