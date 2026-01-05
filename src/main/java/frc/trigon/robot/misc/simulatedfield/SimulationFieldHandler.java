package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * Handles the simulation of game pieces.
 */
public class SimulationFieldHandler {
    private static final ArrayList<SimulatedGamePiece> GAME_PIECES_ON_FIELD = SimulatedGamePieceConstants.STARTING_GAME_PIECES;
    private static Integer HELD_GAME_PIECE_INDEX = null;

    public static ArrayList<SimulatedGamePiece> getSimulatedGamePieces() {
        return GAME_PIECES_ON_FIELD;
    }

    public static boolean isHoldingGamePiece() {
        return HELD_GAME_PIECE_INDEX != null;
    }

    public static void update() {
        updateGamePieces();
        logGamePieces();
    }

    /**
     * Updates the state of all game pieces.
     */
    private static void updateGamePieces() {
        updateGamePiecesPeriodically();
        updateCollection();
        updateEjection();
        updateHeldGamePiecePoses();
    }

    /**
     * Logs the position of all the game pieces.
     */
    private static void logGamePieces() {
        Logger.recordOutput("Poses/GamePieces/GamePieces", mapSimulatedGamePieceListToPoseArray(GAME_PIECES_ON_FIELD));
    }

    private static void updateGamePiecesPeriodically() {
        for (SimulatedGamePiece gamePiece : GAME_PIECES_ON_FIELD)
            gamePiece.updatePeriodically(HELD_GAME_PIECE_INDEX != null && HELD_GAME_PIECE_INDEX == GAME_PIECES_ON_FIELD.indexOf(gamePiece));
    }

    private static void updateCollection() {
        final Pose3d robotPose = new Pose3d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose());
        final Pose3d robotRelativeCollectionPose = new Pose3d();
        final Pose3d collectionPose = robotPose.plus(toTransform(robotRelativeCollectionPose));

        updateFeederCollection(robotPose);

        if (isCollectingGamePiece() && HELD_GAME_PIECE_INDEX == null)
            HELD_GAME_PIECE_INDEX = getIndexOfCollectedGamePiece(collectionPose, GAME_PIECES_ON_FIELD, SimulatedGamePieceConstants.INTAKE_TOLERANCE_METERS);
    }

    private static void updateFeederCollection(Pose3d robotPose) {
        final double distanceFromFeeder = robotPose.toPose2d().getTranslation().getDistance(SimulatedGamePieceConstants.FEEDER_POSITION.get());

        if (isCollectingGamePieceFromFeeder() && HELD_GAME_PIECE_INDEX == null &&
                distanceFromFeeder < SimulatedGamePieceConstants.FEEDER_INTAKE_TOLERANCE_METERS) {
            GAME_PIECES_ON_FIELD.add(new SimulatedGamePiece(new Pose3d(), SimulatedGamePieceConstants.GamePieceType.GAME_PIECE_TYPE));
            HELD_GAME_PIECE_INDEX = GAME_PIECES_ON_FIELD.size() - 1;
        }
    }

    /**
     * Gets the index of the game piece that is being collected.
     *
     * @param collectionPose the pose of the collection mechanism
     * @param gamePieceList  the list of game pieces
     * @return the index of the game piece that is being collected
     */
    private static Integer getIndexOfCollectedGamePiece(Pose3d collectionPose, ArrayList<SimulatedGamePiece> gamePieceList, double intakeTolerance) {
        for (SimulatedGamePiece gamePiece : gamePieceList)
            if (gamePiece.getDistanceFromPoseMeters(collectionPose) <= intakeTolerance)
                return gamePieceList.indexOf(gamePiece);
        return null;
    }

    private static boolean isCollectingGamePiece() {
        return false;//TODO: Implement
    }

    private static boolean isCollectingGamePieceFromFeeder() {
        return false;//TODO: Implement
    }

    private static void updateEjection() {
        if (HELD_GAME_PIECE_INDEX != null && isEjectingGamePiece()) {
            final SimulatedGamePiece heldGamePiece = GAME_PIECES_ON_FIELD.get(HELD_GAME_PIECE_INDEX);
            ejectGamePiece(heldGamePiece);
            HELD_GAME_PIECE_INDEX = null;
        }
    }

    private static void ejectGamePiece(SimulatedGamePiece ejectedGamePiece) {
        final Translation3d robotSelfRelativeVelocity = new Translation3d(RobotContainer.SWERVE.getSelfRelativeVelocity());
        final Translation3d robotRelativeReleaseVelocity = new Translation3d();//TODO:This should be extracted to a method in a different branch...

        ejectedGamePiece.release(robotSelfRelativeVelocity.plus(robotRelativeReleaseVelocity).rotateBy(new Rotation3d(RobotContainer.SWERVE.getHeading())));
    }

    private static boolean isEjectingGamePiece() {
        return false;//TODO: Implement
    }

    /**
     * Updates the position of the held game pieces so that they stay inside the robot.
     */
    private static void updateHeldGamePiecePoses() {
        final Pose3d robotRelativeHeldGamePiecePosition = new Pose3d();
        updateHeldGamePiecePose(robotRelativeHeldGamePiecePosition, GAME_PIECES_ON_FIELD, HELD_GAME_PIECE_INDEX);
    }

    private static void updateHeldGamePiecePose(Pose3d robotRelativeHeldGamePiecePose, ArrayList<SimulatedGamePiece> gamePieceList, Integer heldGamePieceIndex) {
        if (heldGamePieceIndex == null)
            return;

        final SimulatedGamePiece heldGamePiece = gamePieceList.get(heldGamePieceIndex);
        heldGamePiece.updatePose(calculateHeldGamePieceFieldRelativePose(robotRelativeHeldGamePiecePose));
    }

    /**
     * Calculate the position of the held game piece relative to the field.
     *
     * @param robotRelativeHeldGamePiecePosition the position of the held game piece relative to the robot
     * @return the position of the held game piece relative to the field
     */
    private static Pose3d calculateHeldGamePieceFieldRelativePose(Pose3d robotRelativeHeldGamePiecePosition) {
        final Pose3d robotPose3d = new Pose3d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose());
        return robotPose3d.plus(toTransform(robotRelativeHeldGamePiecePosition));
    }

    /**
     * Converts a Pose3d into a Transform3d.
     *
     * @param pose the target Pose3d
     * @return the Transform3d
     */
    private static Transform3d toTransform(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    private static Pose3d[] mapSimulatedGamePieceListToPoseArray(ArrayList<SimulatedGamePiece> gamePieces) {
        final Pose3d[] poses = new Pose3d[gamePieces.size()];
        for (int i = 0; i < poses.length; i++)
            poses[i] = gamePieces.get(i).getPose();
        return poses;
    }
}