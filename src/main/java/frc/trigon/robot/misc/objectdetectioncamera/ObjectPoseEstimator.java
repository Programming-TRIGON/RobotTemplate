package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashMap;

public class ObjectPoseEstimator extends SubsystemBase {
    private final HashMap<Translation2d, Double> knownObjectPositions;
    private final ObjectDetectionCamera camera;
    private final double deletionThresholdSeconds;
    private final SimulatedGamePieceConstants.GamePieceType gamePieceType;
    private final double rotationToTranslation;

    /**
     * Constructs an ObjectPoseEstimator for estimating the positions of objects detected by camera.
     *
     * @param deletionThresholdSeconds the time in seconds after which an object is considered old and removed
     * @param gamePieceType            the type of game piece to track
     * @param camera                   the camera used for detecting objects
     */
    public ObjectPoseEstimator(double deletionThresholdSeconds, DistanceCalculationMethod distanceCalculationMethod,
                               SimulatedGamePieceConstants.GamePieceType gamePieceType,
                               ObjectDetectionCamera camera) {
        this.deletionThresholdSeconds = deletionThresholdSeconds;
        this.gamePieceType = gamePieceType;
        this.camera = camera;
        this.knownObjectPositions = new HashMap<>();
        this.rotationToTranslation = distanceCalculationMethod.rotationToTranslation;
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
        return new ArrayList<>(knownObjectPositions.keySet());
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
        removeObject(getClosestObjectToPosition(robotPose.transformBy(intakeTransform).getTranslation()));
    }

    /**
     * Removes the closest object to a given pose from the list of objects in the pose estimator.
     *
     * @param fieldRelativePose the pose to which the removed object is closest
     */
    public void removeClosestObjectToPose(Pose2d fieldRelativePose) {
        final Translation2d closestObject = getClosestObjectToPosition(fieldRelativePose.getTranslation());
        removeObject(closestObject);
    }

    public void removeClosestObjectToPosition(Translation2d position) {
        final Translation2d closestObject = getClosestObjectToPosition(position);
        removeObject(closestObject);
    }

    /**
     * Removes a specific object from the list of known objects in the pose estimator.
     *
     * @param objectPosition the position of the object to be removed. Must be the precise position as stored in the pose estimator.
     */
    public void removeObject(Translation2d objectPosition) {
        knownObjectPositions.remove(objectPosition);
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
        return getClosestObjectToPosition(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation());
    }


    /**
     * Gets the position of the closest object to a given position on the field.
     *
     * @param position the position to which the returned object is closest
     * @return the closest object's position on the field, or null if no objects are known
     */
    public Translation2d getClosestObjectToPosition(Translation2d position) {
        final Translation2d[] objectsTranslations = knownObjectPositions.keySet().toArray(Translation2d[]::new);
        if (knownObjectPositions.isEmpty())
            return null;
        Translation2d bestObjectTranslation = objectsTranslations[0];
        double closestObjectDistance = position.getDistance(bestObjectTranslation);

        for (int i = 1; i < objectsTranslations.length; i++) {
            final Translation2d currentObjectTranslation = objectsTranslations[i];
            final double currentObjectDistance = position.getDistance(currentObjectTranslation);
            if (currentObjectDistance < closestObjectDistance) {
                closestObjectDistance = currentObjectDistance;
                bestObjectTranslation = currentObjectTranslation;
            }
        }
        return bestObjectTranslation;
    }

    /**
     * Calculates the "distance rating" of an object.
     * The "distance rating" is a unit used to calculate the distance between 2 poses.
     * It factors in both translation and rotation differences by scaling the units depending on the {@link DistanceCalculationMethod}.
     *
     * @param objectTranslation the translation of the object on the field
     * @param pose              the pose to which the distance is measured from
     * @return the objects "distance rating"
     */
    private double calculateObjectDistanceRating(Translation2d objectTranslation, Pose2d pose) {
        final double translationDifference = pose.getTranslation().getDistance(objectTranslation);
        final double xDifference = Math.abs(pose.getX() - objectTranslation.getX());
        final double yDifference = Math.abs(pose.getY() - objectTranslation.getY());
        final double rotationDifferenceDegrees = Math.abs(pose.getRotation().getDegrees() - Math.atan2(yDifference, xDifference));
        return translationDifference * rotationToTranslation + rotationDifferenceDegrees * (1 - rotationToTranslation);
    }

    private void updateObjectPositions() {
        final double currentTimestamp = Timer.getTimestamp();
        for (Translation2d visibleObject : camera.getObjectPositionsOnField(gamePieceType)) {
            Translation2d closestObjectToVisibleObject = new Translation2d();
            double closestObjectToVisibleObjectDistanceMeters = Double.POSITIVE_INFINITY;

            for (Translation2d knownObject : knownObjectPositions.keySet()) {
                final double currentObjectDistanceMeters = visibleObject.getDistance(knownObject);
                if (currentObjectDistanceMeters < closestObjectToVisibleObjectDistanceMeters) {
                    closestObjectToVisibleObjectDistanceMeters = currentObjectDistanceMeters;
                    closestObjectToVisibleObject = knownObject;
                }
            }

            if (closestObjectToVisibleObjectDistanceMeters < ObjectDetectionCameraConstants.TRACKED_OBJECT_TOLERANCE_METERS && knownObjectPositions.get(closestObjectToVisibleObject) != currentTimestamp)
                removeObject(closestObjectToVisibleObject);
            knownObjectPositions.put(visibleObject, currentTimestamp);
        }
    }

    private void removeOldObjects() {
        knownObjectPositions.entrySet().removeIf(entry -> isTooOld(entry.getValue()));
    }

    private boolean isTooOld(double timestamp) {
        return Timer.getTimestamp() - timestamp > deletionThresholdSeconds;
    }

    public enum DistanceCalculationMethod {
        ROTATION(0),
        TRANSLATION(1),
        ROTATION_AND_TRANSLATION(0.1);

        /**
         * The ratio of rotation to translation in the distance rating calculation.
         * A value of 0 means only rotation is considered, 1 means only translation is considered.
         * Values in between are the ratio of rotation to translation in the distance rating calculation.
         * For example, a value of 0.1 means that 9 cm of translation is considered equivalent to 1 degree of rotation.
         */
        final double rotationToTranslation;

        DistanceCalculationMethod(double rotationToTranslation) {
            this.rotationToTranslation = rotationToTranslation;
        }
    }
}