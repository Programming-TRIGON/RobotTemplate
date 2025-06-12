package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;
import org.trigon.utilities.flippable.FlippableRotation2d;

import java.util.function.Supplier;

public class GamePieceAutoDriveCommand extends ParallelCommandGroup {
    private static final ObjectDetectionCamera CAMERA = CameraConstants.OBJECT_DETECTION_CAMERA;
    private final SimulatedGamePieceConstants.GamePieceType targetGamePieceType;
    private Translation2d distanceFromTrackedGamePiece;

    public GamePieceAutoDriveCommand(SimulatedGamePieceConstants.GamePieceType targetGamePieceType) {
        this.targetGamePieceType = targetGamePieceType;
        addCommands(
                new InstantCommand(CAMERA::initializeTracking),
                getTrackGamePieceCommand(),
                GeneralCommands.getContinuousConditionalCommand(
                        getDriveToGamePieceCommand(() -> distanceFromTrackedGamePiece),
                        GeneralCommands.getFieldRelativeDriveCommand(),
                        () -> CAMERA.getTrackedObjectFieldRelativePosition() != null && shouldMoveTowardsGamePiece(distanceFromTrackedGamePiece)
                )
        );
    }

    private Command getTrackGamePieceCommand() {
        return new RunCommand(() -> {
            CAMERA.trackObject(targetGamePieceType);
            distanceFromTrackedGamePiece = calculateDistanceFromTrackedGamePiece();
        });
    }

    public static Translation2d calculateDistanceFromTrackedGamePiece() {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d trackedObjectPositionOnField = CAMERA.getTrackedObjectFieldRelativePosition();
        if (trackedObjectPositionOnField == null)
            return null;

        final Translation2d robotToGamePiece = robotPose.getTranslation().minus(trackedObjectPositionOnField);
        var distanceFromTrackedGamePiece = robotToGamePiece.rotateBy(robotPose.getRotation().unaryMinus());
        Logger.recordOutput("GamePieceAutoDrive/DistanceFromTrackedGamePiece", distanceFromTrackedGamePiece);
        Logger.recordOutput("GamePieceAutoDrive/XDistanceFromTrackedGamePiece", PathPlannerConstants.GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER.getSetpoint().position);
        return distanceFromTrackedGamePiece;
    }

    public static Command getDriveToGamePieceCommand(Supplier<Translation2d> distanceFromTrackedGamePiece) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> PathPlannerConstants.GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.get().getX(), RobotContainer.SWERVE.getSelfRelativeVelocity().vxMetersPerSecond)),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> PathPlannerConstants.GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER.calculate(distanceFromTrackedGamePiece.get().getX()),
                        () -> PathPlannerConstants.GAME_PIECE_AUTO_DRIVE_Y_PID_CONTROLLER.calculate(distanceFromTrackedGamePiece.get().getY()),
                        GamePieceAutoDriveCommand::calculateTargetAngle
                )
        );
    }

    public static boolean shouldMoveTowardsGamePiece(Translation2d distanceFromTrackedGamePiece) {
        return distanceFromTrackedGamePiece != null &&
                (distanceFromTrackedGamePiece.getNorm() > PathPlannerConstants.AUTO_COLLECTION_OPENING_CHECK_DISTANCE_METERS);//TODO: If intake is open
    }

    public static FlippableRotation2d calculateTargetAngle() {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d trackedObjectFieldRelativePosition = CAMERA.getTrackedObjectFieldRelativePosition();
        if (trackedObjectFieldRelativePosition == null)
            return null;

        final Translation2d objectDistanceFromRobot = trackedObjectFieldRelativePosition.minus(robotPose.getTranslation());
        final Rotation2d angularDistanceToObject = new Rotation2d(Math.atan2(objectDistanceFromRobot.getY(), objectDistanceFromRobot.getX()));
        return new FlippableRotation2d(angularDistanceToObject, false);
    }
}