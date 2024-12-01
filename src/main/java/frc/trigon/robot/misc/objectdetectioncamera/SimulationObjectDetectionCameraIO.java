package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * A simulation object detection camera simulates an object detection camera as well as game pieces on the field and allows for interaction with the game pieces.
 */
public class SimulationObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    public static boolean HAS_OBJECTS = true;
    private static final Rotation2d CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(75);
    private static final double
            MAXIMUM_VISIBLE_DISTANCE_METERS = 5,
            MINIMUM_VISIBLE_DISTANCE_METERS = 0.05;
    private static final double PICKING_UP_TOLERANCE_METERS = 0.3;
    private static final double GAME_PIECE_CENTER_DISTANCE_FROM_GROUND = 0;//TODO: Change depending on height of game piece

    private final ArrayList<Translation2d> objectsOnField = new ArrayList<>(List.of(
            new Translation2d(0, 0)// TODO: Set game piece positions
    ));
    private final String hostname;
    private Pose3d[] heldObject = new Pose3d[0];
    private boolean isDelayingEjection = false;

    protected SimulationObjectDetectionCameraIO(String hostname) {
        this.hostname = hostname;
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        final Rotation2d closestObjectYaw = calculateClosestVisibleObjectYaw(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose());
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

    /**
     * Calculates the yaw of the closest visible object relative to the camera by calculating the yaw of all objects visible to the camera and returning the one with the smallest yaw deviation from the camera's center.
     *
     * @param robotPose the pose of the robot on the field
     * @return the yaw of the closest visible object
     */
    private Rotation2d calculateClosestVisibleObjectYaw(Pose2d robotPose) {
        Translation2d closestObject = null;
        Rotation2d closestObjectYaw = null;
        double closestObjectDistance = Double.POSITIVE_INFINITY;

        final List<Translation2d> visibleObjectsPlacements = calculateVisibleObjectsPlacements(robotPose);

        for (Translation2d objectPlacement : visibleObjectsPlacements) {
            final double robotDistanceToObject = calculateObjectDistance(objectPlacement, robotPose);
            if (robotDistanceToObject < closestObjectDistance) {
                closestObject = objectPlacement;
                closestObjectYaw = calculateCameraYawToObject(objectPlacement, robotPose).minus(robotPose.getRotation());
                closestObjectDistance = robotDistanceToObject;
            }
        }

        logClosestObjectPlacement(closestObject);
        return closestObjectYaw;
    }

    /**
     * Calculates the placements of all visible objects by checking if they are within range and within the horizontal FOV.
     *
     * @param robotPose the position of the robot on the field
     * @return the placements of the visible objects
     */
    private List<Translation2d> calculateVisibleObjectsPlacements(Pose2d robotPose) {
        final List<Translation2d> visibleObjects = new ArrayList<>();
        int currentIndex = 0;

        for (Translation2d currentObject : objectsOnField) {
            final Rotation2d cameraYawToObject = calculateCameraYawToObject(currentObject, robotPose);
            if (!isWithinHorizontalFOV(cameraYawToObject, robotPose) || !isWithinDistance(objectsOnField.get(0), robotPose))
                continue;
            visibleObjects.add(currentIndex, currentObject);
            currentIndex++;
        }
        return visibleObjects;
    }

    /**
     * Calculates the difference in yaw from an object to the camera's yaw.
     *
     * @param objectPlacement the placement of the object on the field
     * @param robotPose       the position of the robot on the field
     * @return the yaw of the object relative to the camera
     */
    private Rotation2d calculateCameraYawToObject(Translation2d objectPlacement, Pose2d robotPose) {
        final Translation2d robotToObject = objectPlacement.minus(robotPose.getTranslation());
        return robotToObject.getAngle();
    }

    /**
     * Checks if an object is within the field-of-view of the camera.
     *
     * @param objectYaw the yaw of the object relative to the camera
     * @param robotPose the position of the robot on the field
     * @return if the object is within the field-of-view of the camera
     */
    private boolean isWithinHorizontalFOV(Rotation2d objectYaw, Pose2d robotPose) {
        return Math.abs(objectYaw.minus(robotPose.getRotation()).getRadians()) <= CAMERA_HORIZONTAL_FOV.getRadians() / 2;
    }

    /**
     * Checks if an object is within the distance range of the camera.
     * The distance range is set at the top of this class as {@link #MINIMUM_VISIBLE_DISTANCE_METERS} and {@link #MINIMUM_VISIBLE_DISTANCE_METERS}.
     *
     * @param objectPlacement the placement of the object on the field
     * @param robotPose       the position of the robot on the field
     * @return if the object is withing the distance range of the camera
     */
    private boolean isWithinDistance(Translation2d objectPlacement, Pose2d robotPose) {
        final double robotToObjectDistanceMeters = calculateObjectDistance(objectPlacement, robotPose);
        return robotToObjectDistanceMeters <= MAXIMUM_VISIBLE_DISTANCE_METERS && robotToObjectDistanceMeters >= MINIMUM_VISIBLE_DISTANCE_METERS;
    }

    /**
     * Calculates the distance between the robot and an object.
     *
     * @param objectPlacement the placement of the object on the field
     * @param robotPose       the position of the robot on the field
     * @return the distance between the robot and the object
     */
    private double calculateObjectDistance(Translation2d objectPlacement, Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(objectPlacement);
    }

    /**
     * Logs the closest object's placement on the field.
     *
     * @param objectPlacement the placement of the closest object
     */
    private void logClosestObjectPlacement(Translation2d objectPlacement) {
        if (objectPlacement != null)
            Logger.recordOutput(hostname + "/ClosestObject", objectPlacement);
        else
            Logger.recordOutput(hostname + "/ClosestObject", new Translation2d[0]);
    }

    /**
     * Updates the state of the held game piece (whether it is collecting, ejecting, etc.)
     */
    private void updateHeldGamePiece() {
        updateObjectCollection();
        updateObjectEjection();
        updateHeldObjectPose();
    }

    /**
     * Handles when a game piece should collect by checking the state of the necessary subsystems of the robot and the position of the robot relative to the game piees.
     * Also regenerates picked up game pieces.
     */
    private void updateObjectCollection() {
        if (heldObject.length > 0 || !isCollecting())
            return;
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final Translation2d robotTranslation = robotPose.getTranslation();
        for (Translation2d objectPlacement : objectsOnField) {
            if (objectPlacement.getDistance(robotTranslation) <= PICKING_UP_TOLERANCE_METERS) {
                heldObject = new Pose3d[]{calculateHeldObjectPose(robotPose)};
                objectsOnField.remove(objectPlacement);
                GeneralCommands.getDelayedCommand(10, () -> objectsOnField.add(objectPlacement)).schedule();
                break;
            }
        }
    }

    /**
     * Handles when a game piece should eject from the robot by checking the state of the necessary subsystems.
     */
    private void updateObjectEjection() {
        if (heldObject.length > 0 || !isEjecting() || isDelayingEjection)
            return;
        isDelayingEjection = true;
        GeneralCommands.getDelayedCommand(0.04, () -> {
            heldObject = null;
            isDelayingEjection = false;
        }).schedule();
    }

    /**
     * Updates the position of the held game piece so that it stays inside the robot.
     */
    private void updateHeldObjectPose() {
        if (heldObject.length == 0)
            return;
        heldObject = new Pose3d[]{calculateHeldObjectPose(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose())};
    }

    private boolean isCollecting() {
        return true;//TODO: Implement for when a game piece can enter the robot
    }

    private boolean isEjecting() {
        return false;//TODO: Implement for when a game piece should exit the robot
    }

    /**
     * Calculate the position of the game piece relative to the field.
     *
     * @param robotPose the position of the robot on the field
     * @return the position of the game piece relative to the field
     */
    private Pose3d calculateHeldObjectPose(Pose2d robotPose) {
        final Pose3d robotPose3d = new Pose3d(robotPose);
        final Pose3d robotRelativeHeldGamePiecePosition = new Pose3d();// TODO:Set position
        return robotPose3d.plus(toTransform(robotRelativeHeldGamePiecePosition));
    }

    /**
     * Changes a Pose3d into a Transform3d.
     *
     * @param pose the target Pose3d
     * @return the Transform3d
     */
    private Transform3d toTransform(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Logs the position of all the game pieces on the field and in the robot.
     */
    private void logGamePieces() {
        Logger.recordOutput("Poses/GamePieces/HeldGamePiece", heldObject);
        Logger.recordOutput("Poses/GamePieces/ObjectsOnField", toPosesArray(objectsOnField));
    }

    /**
     * Changes a list of Translation2ds to an array of Pose3ds with the height of {@link #GAME_PIECE_CENTER_DISTANCE_FROM_GROUND}.
     *
     * @param translationsList the list of Translation2ds
     * @return the array of Pose3ds
     */
    private Pose3d[] toPosesArray(List<Translation2d> translationsList) {
        final Pose3d[] posesArray = new Pose3d[translationsList.size()];
        for (int i = 0; i < translationsList.size(); i++) {
            final Translation2d translation = translationsList.get(i);
            posesArray[i] = new Pose3d(translation.getX(), translation.getY(), GAME_PIECE_CENTER_DISTANCE_FROM_GROUND, new Rotation3d());
        }
        return posesArray;
    }
}