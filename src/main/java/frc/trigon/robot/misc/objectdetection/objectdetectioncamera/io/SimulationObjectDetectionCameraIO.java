package frc.trigon.robot.misc.objectdetection.objectdetectioncamera.io;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.objectdetection.ObjectDetectionConstants;
import frc.trigon.robot.misc.objectdetection.objectdetectioncamera.ObjectDetectionCameraIO;
import frc.trigon.robot.misc.objectdetection.objectdetectioncamera.ObjectDetectionCameraInputsAutoLogged;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePiece;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class SimulationObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private static final Rotation2d
            CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(75),
            CAMERA_VERTICAL_FOV = Rotation2d.fromDegrees(45);

    private final String hostname;
    private final Transform3d robotCenterToCamera;

    public SimulationObjectDetectionCameraIO(String hostname, Transform3d robotCenterToCamera) {
        this.hostname = hostname;
        this.robotCenterToCamera = robotCenterToCamera;
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Pose3d cameraPose = new Pose3d(robotPose).plus(robotCenterToCamera);
        final ArrayList<Pair<SimulatedGamePiece, Rotation3d>>[] visibleGamePieces = calculateAllVisibleGamePieces(cameraPose);

        boolean hasAnyObject = false;
        for (int i = 0; i < ObjectDetectionConstants.NUMBER_OF_GAME_PIECE_TYPES; i++) {
            inputs.hasObject[i] = !visibleGamePieces[i].isEmpty();
            if (inputs.hasObject[i])
                hasAnyObject = true;
        }

        if (hasAnyObject) {
            updateHasNewResultInputs(inputs, visibleGamePieces);
            return;
        }

        updateNoNewResultInputs(inputs);
    }

    private ArrayList<Pair<SimulatedGamePiece, Rotation3d>>[] calculateAllVisibleGamePieces(Pose3d cameraPose) {
        final ArrayList<Pair<SimulatedGamePiece, Rotation3d>>[] visibleGamePieces = new ArrayList[ObjectDetectionConstants.NUMBER_OF_GAME_PIECE_TYPES];
        for (int i = 0; i < visibleGamePieces.length; i++)
            visibleGamePieces[i] = calculateVisibleGamePiecesRotations(cameraPose, i);
        return visibleGamePieces;
    }

    private void updateNoNewResultInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        inputs.hasObject = new boolean[ObjectDetectionConstants.NUMBER_OF_GAME_PIECE_TYPES];
        inputs.visibleObjectRotations = new Rotation3d[ObjectDetectionConstants.NUMBER_OF_GAME_PIECE_TYPES][0];
    }

    private void updateHasNewResultInputs(ObjectDetectionCameraInputsAutoLogged inputs, ArrayList<Pair<SimulatedGamePiece, Rotation3d>>[] visibleGamePieces) {
        for (int i = 0; i < visibleGamePieces.length; i++) {
            inputs.visibleObjectRotations[i] = new Rotation3d[visibleGamePieces[i].size()];
            for (int j = 0; j < visibleGamePieces[i].size(); j++)
                inputs.visibleObjectRotations[i][j] = visibleGamePieces[i].get(j).getSecond();
        }

        inputs.latestResultTimestamp = Timer.getTimestamp();

        logVisibleGamePieces(visibleGamePieces);
    }

    /**
     * Calculates the placements of all visible objects by checking if they are within range and within the horizontal FOV.
     *
     * @param cameraPose the position of the robot on the field
     * @param objectID   the ID of the object to check for visibility
     * @return the placements of the visible objects, as a pair of the object and the rotation of the object relative to the camera
     */
    private ArrayList<Pair<SimulatedGamePiece, Rotation3d>> calculateVisibleGamePiecesRotations(Pose3d cameraPose, int objectID) {
        final ArrayList<SimulatedGamePiece> gamePiecesOnField = SimulationFieldHandler.getSimulatedGamePieces();
        final ArrayList<Pair<SimulatedGamePiece, Rotation3d>> visibleObjects = new ArrayList<>();
        for (SimulatedGamePiece currentObject : gamePiecesOnField) {
            if (currentObject.isScored())
                continue;
            final Rotation3d cameraAngleToObject = calculateCameraAngleToObject(currentObject.getPose(), cameraPose);

            if (isObjectWithinFOV(cameraAngleToObject))
                visibleObjects.add(new Pair<>(currentObject, cameraAngleToObject));
        }

        return visibleObjects;
    }

    private Rotation3d calculateCameraAngleToObject(Pose3d objectPose, Pose3d cameraPose) {
        final Translation3d cameraPosition = cameraPose.getTranslation();
        Translation3d objectPosition = objectPose.getTranslation();
        if (objectPose.getRotation().getZ() < 0.2)
            objectPosition = objectPosition.minus(new Translation3d(0, 0, objectPosition.getZ()));

        final Translation3d difference = cameraPosition.minus(objectPosition);
        final Rotation3d differenceAsAngle = getAngle(difference);


        return differenceAsAngle.minus(cameraPose.getRotation());
    }

    private Rotation3d getAngle(Translation3d translation) {
        return new Rotation3d(0, getPitch(translation).getRadians(), getYaw(translation).getRadians());
    }

    /**
     * Extracts the yaw off of a 3d vector.
     *
     * @param vector the vector to extract the yaw from
     * @return the yaw of the vector
     */
    private Rotation2d getYaw(Translation3d vector) {
        return new Rotation2d(Math.atan2(-vector.getY(), -vector.getX()));
    }

    /**
     * Extracts the pitch off of a 3d vector.
     *
     * @param vector the vector to extract the pitch from
     * @return the pitch of the vector
     */
    private Rotation2d getPitch(Translation3d vector) {
        return new Rotation2d(Math.atan2(vector.getZ(), Math.hypot(vector.getX(), vector.getY())));
    }

    /**
     * Checks if an object is within the field-of-view of the camera.
     *
     * @param objectRotation the rotation of the object relative to the camera
     * @return if the object is within the field-of-view of the camera
     */
    private boolean isObjectWithinFOV(Rotation3d objectRotation) {
        return Math.abs(objectRotation.getZ()) <= CAMERA_HORIZONTAL_FOV.getRadians() / 2 &&
                Math.abs(objectRotation.getY()) <= CAMERA_VERTICAL_FOV.getRadians() / 2;
    }

    private void logVisibleGamePieces(ArrayList<Pair<SimulatedGamePiece, Rotation3d>>[] visibleGamePieces) {
        for (int i = 0; i < visibleGamePieces.length; i++) {
            final String gamePieceTypeName = SimulatedGamePieceConstants.GamePieceType.getNameFromID(i);
            Logger.recordOutput(hostname + "/Visible" + gamePieceTypeName + "poses", mapSimulatedGamePieceListToPoseArray(visibleGamePieces[i]));
        }
    }

    private Pose3d[] mapSimulatedGamePieceListToPoseArray(ArrayList<Pair<SimulatedGamePiece, Rotation3d>> gamePieces) {
        final Pose3d[] poses = new Pose3d[gamePieces.size()];
        for (int i = 0; i < poses.length; i++)
            poses[i] = gamePieces.get(i).getFirst().getPose();

        return poses;
    }
}