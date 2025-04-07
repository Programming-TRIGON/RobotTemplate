package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.objectdetectioncamera.io.PhotonObjectDetectionCameraIO;
import frc.trigon.robot.misc.objectdetectioncamera.io.SimulationObjectDetectionCameraIO;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;

/**
 * An object detection camera is a class that represents a camera that detects objects other than apriltags, most likely game pieces.
 */
public class ObjectDetectionCamera extends SubsystemBase {

    private final ObjectDetectionCameraInputsAutoLogged objectDetectionCameraInputs = new ObjectDetectionCameraInputsAutoLogged();
    private final ObjectDetectionCameraIO objectDetectionCameraIO;
    private final String hostname;
    private final Transform3d robotCenterToCamera;
    private Translation2d trackedObjectFieldRelativePosition = new Translation2d();

    public ObjectDetectionCamera(String hostname, Transform3d robotCenterToCamera) {
        this.hostname = hostname;
        this.robotCenterToCamera = robotCenterToCamera;
        this.objectDetectionCameraIO = generateIO(hostname, robotCenterToCamera);
    }

    @Override
    public void periodic() {
        objectDetectionCameraIO.updateInputs(objectDetectionCameraInputs);
        Logger.processInputs(hostname, objectDetectionCameraInputs);
    }

    /**
     * Calculates the position of the tracked object on the field.
     * This assumes the object is on the ground.
     *
     * @return the tracked object's 2D position on the field (z is assumed to be 0)
     */
    public Translation2d getTrackedObjectFieldRelativePosition() {
        return trackedObjectFieldRelativePosition;
    }

    /**
     * Calculates the position of the best object on the field from the 3D rotation of the object relative to the camera.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @return the best object's 2D position on the field (z is assumed to be 0)
     */
    public Translation2d calculateBestObjectPositionOnField(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        final Translation2d[] targetObjectsTranslation = getObjectPositionsOnField(targetGamePiece);
        final Translation2d currentRobotTranslation = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        if (targetObjectsTranslation.length == 0)
            return null;
        Translation2d bestObjectTranslation = targetObjectsTranslation[0];

        for (int i = 1; i < targetObjectsTranslation.length; i++) {
            final Translation2d currentObjectTranslation = targetObjectsTranslation[i];
            final double bestObjectDifference = currentRobotTranslation.getDistance(bestObjectTranslation);
            final double currentObjectDifference = currentRobotTranslation.getDistance(currentObjectTranslation);
            if (currentObjectDifference < bestObjectDifference)
                bestObjectTranslation = currentObjectTranslation;
        }
        return bestObjectTranslation;
    }

    /**
     * Calculates the position of the object on the field from the 3D rotation of the object relative to the camera.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @param objectRotation the object's 3D rotation relative to the camera
     * @return the object's 2D position on the field (z is assumed to be 0)
     */
    public Translation2d calculateObjectPositionFromRotation(Rotation3d objectRotation) {
        final Pose2d robotPoseAtResultTimestamp = RobotContainer.POSE_ESTIMATOR.getEstimatedPoseAtTimestamp(objectDetectionCameraInputs.latestResultTimestamp);
        if (robotPoseAtResultTimestamp == null)
            return new Translation2d();
        final Pose3d cameraPose = new Pose3d(robotPoseAtResultTimestamp).plus(robotCenterToCamera);
        objectRotation = new Rotation3d(objectRotation.getX(), -objectRotation.getY(), objectRotation.getZ());
        final Pose3d objectRotationStart = cameraPose.plus(new Transform3d(0, 0, 0, objectRotation));

        final double cameraZ = cameraPose.getTranslation().getZ();
        final double objectPitchSin = Math.sin(objectRotationStart.getRotation().getY());
        final double xTransform = cameraZ / objectPitchSin;
        final Transform3d objectRotationStartToGround = new Transform3d(xTransform, 0, 0, new Rotation3d());

        return objectRotationStart.transformBy(objectRotationStartToGround).getTranslation().toTranslation2d();
    }

    public void initializeTracking() {
        trackedObjectFieldRelativePosition = null;
    }

    /**
     * Starts tracking the best visible target of the target ID and remains tracking that target until it is no longer visible.
     * Tracking an object is locking on to one target and allows for you to remain locked on to one target even when there are more objects visible.
     * This is used when there is more than one visible object of the target ID and the best target might change as the robot moves.
     * When no objects are visible, the tracking resets to the best target the next time an object of the target ID is visible.
     * This should be called periodically.
     *
     * @param targetGamePiece the type of game piece to track
     */
    public void trackObject(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        if (!hasTargets(targetGamePiece))
            return;

        updateTrackedObjectPose(targetGamePiece);
        if (trackedObjectFieldRelativePosition != null)
            Logger.recordOutput("ObjectDetectionCamera/TrackedObject", new Translation3d(trackedObjectFieldRelativePosition));
    }

    public boolean hasTargets(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        return objectDetectionCameraInputs.hasTarget[targetGamePiece.id];
    }

    public Rotation3d[] getTargetObjectsRotations(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        return objectDetectionCameraInputs.visibleObjectRotations[targetGamePiece.id];
    }

    private void updateTrackedObjectPose(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        if (trackedObjectFieldRelativePosition == null) {
            trackedObjectFieldRelativePosition = calculateBestObjectPositionOnField(targetGamePiece);
            return;
        }

        Translation2d closestObjectToTrackedObjectPositionOnField = new Translation2d();
        double closestObjectToTrackedObjectDistanceMeters = Double.POSITIVE_INFINITY;

        for (Translation2d targetObjectPositionOnField : getObjectPositionsOnField(targetGamePiece)) {
            final double currentToTrackedObjectDistanceMeters = targetObjectPositionOnField.getDistance(trackedObjectFieldRelativePosition);
            if (currentToTrackedObjectDistanceMeters < closestObjectToTrackedObjectDistanceMeters) {
                closestObjectToTrackedObjectDistanceMeters = currentToTrackedObjectDistanceMeters;
                closestObjectToTrackedObjectPositionOnField = targetObjectPositionOnField;
            }
        }

        if (closestObjectToTrackedObjectDistanceMeters <= ObjectDetectionCameraConstants.TRACKED_OBJECT_TOLERANCE_METERS)
            trackedObjectFieldRelativePosition = closestObjectToTrackedObjectPositionOnField;
    }

    private Translation2d[] getObjectPositionsOnField(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        final Rotation3d[] visibleObjectsRotations = getTargetObjectsRotations(targetGamePiece);
        final Translation2d[] objectsPositionsOnField = new Translation2d[visibleObjectsRotations.length];

        for (int i = 0; i < visibleObjectsRotations.length; i++)
            objectsPositionsOnField[i] = calculateObjectPositionFromRotation(visibleObjectsRotations[i]);

        Logger.recordOutput("ObjectDetectionCamera/Visible" + targetGamePiece.name(), objectsPositionsOnField);
        return objectsPositionsOnField;
    }

    private ObjectDetectionCameraIO generateIO(String hostname, Transform3d robotCenterToCamera) {
        if (RobotHardwareStats.isReplay())
            return new ObjectDetectionCameraIO();
        if (RobotHardwareStats.isSimulation())
            return new SimulationObjectDetectionCameraIO(hostname, robotCenterToCamera);
        return new PhotonObjectDetectionCameraIO(hostname);
    }
}