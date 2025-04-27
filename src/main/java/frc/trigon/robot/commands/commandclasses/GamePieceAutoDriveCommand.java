package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.flippable.FlippableRotation2d;

import java.util.function.Supplier;

public class GamePieceAutoDriveCommand extends ParallelCommandGroup {
    private static final double AUTO_COLLECTION_OPENING_CHECK_DISTANCE_METERS = 2;
    private static final PIDController Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.5, 0, 0) :
            new PIDController(0.3, 0, 0.03);
    private static final ProfiledPIDController X_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
            new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(2.65, 5.5));
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
        Logger.recordOutput("GamePieceAutoDrive/XDistanceFromTrackedGamePiece", X_PID_CONTROLLER.getSetpoint().position);
        return distanceFromTrackedGamePiece;
    }

    public static Command getDriveToGamePieceCommand(Supplier<Translation2d> distanceFromTrackedGamePiece) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> X_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.get().getX(), RobotContainer.SWERVE.getSelfRelativeVelocity().vxMetersPerSecond)),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> X_PID_CONTROLLER.calculate(distanceFromTrackedGamePiece.get().getX()),
                        () -> Y_PID_CONTROLLER.calculate(distanceFromTrackedGamePiece.get().getY()),
                        GamePieceAutoDriveCommand::calculateTargetAngle
                )
        );
    }

    public static boolean shouldMoveTowardsGamePiece(Translation2d distanceFromTrackedGamePiece) {
        return distanceFromTrackedGamePiece != null &&
                (distanceFromTrackedGamePiece.getNorm() > AUTO_COLLECTION_OPENING_CHECK_DISTANCE_METERS);//TODO: If intake is open
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