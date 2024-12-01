package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class SimulationObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    public static boolean HAS_OBJECTS = true;
    private static final Rotation2d CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(75);
    private static final double
            MAXIMUM_VISIBLE_DISTANCE_METERS = 5,
            MINIMUM_VISIBLE_DISTANCE_METERS = 0.05;
    private static final double PICKING_UP_TOLERANCE_METERS = 0.3;

    private final ArrayList<Translation2d> objectsOnField = new ArrayList<>(List.of(
            new Translation2d(0, 0)// TODO: Set game piece positions
    ));
    private final String hostname;
    private Pose3d heldObject = null;
    private boolean isDelayingEjection = false;

    protected SimulationObjectDetectionCameraIO(String hostname) {
        this.hostname = hostname;
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        final Rotation2d closestObjectYaw = getClosestVisibleObjectYaw(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose());
        if (closestObjectYaw == null) {
            inputs.hasTargets = false;
        } else {
            inputs.hasTargets = true;
            inputs.visibleObjectsYaw = new Rotation2d[]{closestObjectYaw};
        }

        updateHeldGamePiece();

        HAS_OBJECTS = heldObject != null;
        logGamePieces();
    }

    private Rotation2d getClosestVisibleObjectYaw(Pose2d robotPose) {
        Translation2d closestObject = null;
        Rotation2d closestObjectYaw = null;
        double closestObjectDistance = Double.POSITIVE_INFINITY;

        List<Translation2d> visibleObjects = getVisibleObjects(robotPose);

        for (Translation2d objectPlacement : visibleObjects) {
            final double robotDistanceToObject = getObjectDistance(objectPlacement, robotPose);
            if (robotDistanceToObject < closestObjectDistance) {
                closestObject = objectPlacement;
                closestObjectYaw = getAngleToObject(objectPlacement, robotPose).minus(robotPose.getRotation());
                closestObjectDistance = robotDistanceToObject;
            }
        }

        logObjectPlacement(closestObject);
        return closestObjectYaw;
    }

    private List<Translation2d> getVisibleObjects(Pose2d robotPose) {
        final List<Translation2d> visibleObjects = new ArrayList<>();
        int currentIndex = 0;

        for (Translation2d currentObject : objectsOnField) {
            final Rotation2d angleToObject = getAngleToObject(currentObject, robotPose);
            if (!isWithinHorizontalFOV(angleToObject, robotPose) || !isWithinDistance(objectsOnField.get(0), robotPose))
                continue;
            visibleObjects.add(currentIndex, currentObject);
            currentIndex++;
        }
        return visibleObjects;
    }

    private Rotation2d getAngleToObject(Translation2d objectPlacement, Pose2d robotPose) {
        final Translation2d robotToObject = objectPlacement.minus(robotPose.getTranslation());
        return Rotation2d.fromRadians(Math.atan2(robotToObject.getY(), robotToObject.getX()));
    }

    private boolean isWithinHorizontalFOV(Rotation2d objectYaw, Pose2d robotPose) {
        return Math.abs(objectYaw.minus(robotPose.getRotation()).getRadians()) <= CAMERA_HORIZONTAL_FOV.getRadians() / 2;
    }

    private boolean isWithinDistance(Translation2d objectPlacement, Pose2d robotPose) {
        final double robotToObjectDistanceMeters = getObjectDistance(objectPlacement, robotPose);
        return robotToObjectDistanceMeters <= MAXIMUM_VISIBLE_DISTANCE_METERS && robotToObjectDistanceMeters >= MINIMUM_VISIBLE_DISTANCE_METERS;
    }

    private double getObjectDistance(Translation2d objectPlacement, Pose2d robotPose) {
        final Translation2d robotToObject = objectPlacement.minus(robotPose.getTranslation());
        return robotToObject.getNorm();
    }

    private void logObjectPlacement(Translation2d objectPlacement) {
        if (objectPlacement != null)
            Logger.recordOutput(hostname + "/ClosestObject", objectPlacement);
        else
            Logger.recordOutput(hostname + "/ClosestObject", new Translation2d[0]);
    }

    private void updateHeldGamePiece() {
        updateObjectCollection();
        updateObjectEjection();
        updateHeldObjectPose();
    }

    private void updateObjectCollection() {
        if (heldObject != null || !isCollecting())
            return;
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final Translation2d robotTranslation = robotPose.getTranslation();
        for (Translation2d objectPlacement : objectsOnField) {
            if (objectPlacement.getDistance(robotTranslation) <= PICKING_UP_TOLERANCE_METERS) {
                heldObject = getHeldObjectPose(robotPose);
                objectsOnField.remove(objectPlacement);
                GeneralCommands.getDelayedCommand(10, () -> objectsOnField.add(objectPlacement)).schedule();
                break;
            }
        }
    }

    private void updateObjectEjection() {
        if (heldObject == null || !isEjecting() || isDelayingEjection)
            return;
        isDelayingEjection = true;
        GeneralCommands.getDelayedCommand(0.04, () -> {
            heldObject = null;
            isDelayingEjection = false;
        }).schedule();
    }

    private void updateHeldObjectPose() {
        if (heldObject == null)
            return;
        heldObject = getHeldObjectPose(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose());
    }

    private boolean isCollecting() {
        return true;//TODO: Implement for when a game piece can enter the robot
    }

    private boolean isEjecting() {
        return false;//TODO: Implement for when a game piece should exit the robot
    }

    private Pose3d getHeldObjectPose(Pose2d robotPose) {
        final Pose3d robotPose3d = new Pose3d(robotPose);
        final Pose3d robotRelativeHeldGamePiecePosition = new Pose3d();// TODO:Set position
        return robotPose3d.plus(toTransform(robotRelativeHeldGamePiecePosition));
    }

    private Transform3d toTransform(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    private void logGamePieces() {
        Logger.recordOutput("Poses/GamePieces/HeldGamePiece", heldObject);
        Logger.recordOutput("Poses/GamePieces/ObjectsOnField", toPosesArray(objectsOnField));
    }

    private Pose3d[] toPosesArray(List<Translation2d> translationsList) {
        final Pose3d[] posesArray = new Pose3d[translationsList.size()];
        for (int i = 0; i < translationsList.size(); i++) {
            final Translation2d translation = translationsList.get(i);
            posesArray[i] = new Pose3d(translation.getX(), translation.getY(), 0.1, new Rotation3d());
        }
        return posesArray;
    }
}