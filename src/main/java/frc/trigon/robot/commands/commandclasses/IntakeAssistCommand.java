package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * A command class to assist in the intaking of a game piece.
 */
public class IntakeAssistCommand extends ParallelCommandGroup {
    private static final ProfiledPIDController
            X_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
            new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(2.65, 5.5)),
            Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
                    new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
                    new ProfiledPIDController(0.3, 0, 0.03, new TrapezoidProfile.Constraints(2.65, 5.5)),
            THETA_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
                    new ProfiledPIDController(0.4, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
                    new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(2.65, 5.5));
    private Translation2d distanceFromTrackedGamePiece;

    /**
     * Creates a new intake assist command that assists the axes the user inputs.
     *
     * @param xAssist     the assist to apply to the X-axis (Forward and backwards)
     * @param yAssist     the assist to apply to the Y-axis (Sideways)
     * @param thetaAssist the assist to apply to the rotation
     */
    public IntakeAssistCommand(AssistAxis xAssist, AssistAxis yAssist, AssistAxis thetaAssist) {
        addCommands(
                new InstantCommand(this::resetPIDControllers).andThen(
                        new WaitUntilCommand(this::hasNoTrackedGamePiece),
                        new WaitUntilCommand(() -> !hasNoTrackedGamePiece())
                ).repeatedly(),
                new RunCommand(this::trackGamePiece),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> calculateTranslationPower(true, xAssist),
                        () -> calculateTranslationPower(false, yAssist),
                        () -> calculateThetaPower(thetaAssist)
                )
        );
    }

    private void resetPIDControllers() {
        if (hasNoTrackedGamePiece())
            return;

        X_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getX(), RobotContainer.SWERVE.getSelfRelativeVelocity().getX());
        Y_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getY(), RobotContainer.SWERVE.getSelfRelativeVelocity().getY());
        THETA_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getAngle().getRadians(), RobotContainer.SWERVE.getRotationalVelocityRadiansPerSecond());
    }

    private double calculateTranslationPower(boolean isXAxis, AssistAxis assistAxis) {
        final Translation2d selfRelativeJoystickValue = getSelfRelativeJoystickPosition();
        final double joystickValue = isXAxis ? selfRelativeJoystickValue.getX() : selfRelativeJoystickValue.getY();

        if (hasNoTrackedGamePiece())
            return joystickValue;

        final double
                pidOutput = clampToOutputRange(isXAxis ?
                X_PID_CONTROLLER.calculate(distanceFromTrackedGamePiece.getX()) :
                Y_PID_CONTROLLER.calculate(distanceFromTrackedGamePiece.getY()));
        final double joystickNorm = selfRelativeJoystickValue.getNorm();

        return assistAxis.calculatePower(pidOutput, joystickValue, joystickNorm);
    }

    private double calculateThetaPower(AssistAxis assistAxis) {
        final double joystickValue = OperatorConstants.DRIVER_CONTROLLER.getRightX();

        if (hasNoTrackedGamePiece())
            return joystickValue;

        return calculateThetaAssistPower(assistAxis, joystickValue, distanceFromTrackedGamePiece.getAngle().plus(Rotation2d.k180deg).unaryMinus());
    }

    private double calculateThetaAssistPower(AssistAxis assistAxis, double joystickValue, Rotation2d thetaOffset) {
        final double
                pidOutput = clampToOutputRange(THETA_PID_CONTROLLER.calculate(thetaOffset.getRadians())),
                joystickNorm = Math.hypot(OperatorConstants.DRIVER_CONTROLLER.getRightX(), OperatorConstants.DRIVER_CONTROLLER.getRightY());

        return assistAxis.calculatePower(pidOutput, joystickValue, joystickNorm);
    }

    private boolean hasNoTrackedGamePiece() {
        return distanceFromTrackedGamePiece == null;
    }

    private void trackGamePiece() {
        distanceFromTrackedGamePiece = RobotContainer.OBJECT_POSE_ESTIMATOR.hasObject() ?
                calculateDistanceFromBestGamePiece() :
                null;
    }

    private double clampToOutputRange(double value) {
        return MathUtil.clamp(value, -1, 1);
    }

    private Translation2d calculateDistanceFromBestGamePiece() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d bestGamePieceFieldRelativePosition = getBestGamePieceFieldRelativePosition(robotPose.getTranslation());

        if (bestGamePieceFieldRelativePosition == null)
            return null;

        final Translation2d bestGamePieceDistanceFromRobot = robotPose.getTranslation().minus(bestGamePieceFieldRelativePosition);
        final Translation2d bestGamePieceOffsetFromRobot = bestGamePieceDistanceFromRobot.rotateBy(robotPose.getRotation().unaryMinus());
        Logger.recordOutput("IntakeAssist/TargetGamePieceOffset", bestGamePieceOffsetFromRobot);
        return bestGamePieceOffsetFromRobot;
    }

    private Translation2d getBestGamePieceFieldRelativePosition(Translation2d robotPosition) {
        final ArrayList<Translation2d> gamePiecePositionsOnField = getGamePiecesInAssistFOV(robotPosition);
        return getClosestGamePieceInDesiredVelocityDirection(robotPosition, gamePiecePositionsOnField);
    }

    private ArrayList<Translation2d> getGamePiecesInAssistFOV(Translation2d robotPosition) {
        final ArrayList<Translation2d> gamePiecePositionsOnField = RobotContainer.OBJECT_POSE_ESTIMATOR.getObjectsOnField();
        gamePiecePositionsOnField.removeIf(
                gamePiecePosition -> !isGamePieceWithinAssistFOV(robotPosition, gamePiecePosition)
        );

        return gamePiecePositionsOnField;
    }

    private Translation2d getClosestGamePieceInDesiredVelocityDirection(Translation2d robotPosition, ArrayList<Translation2d> gamePiecePositionsOnField) {
        Translation2d closestGamePieceFieldRelativePosition = null;
        double distanceFromClosestGamePiece = Double.POSITIVE_INFINITY;

        for (Translation2d gamePieceFieldRelativePosition : gamePiecePositionsOnField) {
            if (!isDrivingTowardsGamePiece(gamePieceFieldRelativePosition, robotPosition))
                continue;

            final double distanceFromCurrentGamePiece = gamePieceFieldRelativePosition.getDistance(robotPosition);
            if (distanceFromCurrentGamePiece < distanceFromClosestGamePiece) {
                closestGamePieceFieldRelativePosition = gamePieceFieldRelativePosition;
                distanceFromClosestGamePiece = distanceFromCurrentGamePiece;
            }
        }

        return closestGamePieceFieldRelativePosition;
    }

    private boolean isGamePieceWithinAssistFOV(Translation2d robotPosition, Translation2d gamePiecePosition) {
        final double gamePieceDistance = robotPosition.getDistance(gamePiecePosition);
        final Rotation2d scaledMaximumAssistAngle = OperatorConstants.INTAKE_ASSIST_MAXIMUM_ANGLE_FROM_GAME_PIECE.times(1 / gamePieceDistance);

        return Math.abs(getSelfRelativeJoystickPosition().getAngle().getRadians()) < Math.abs(scaledMaximumAssistAngle.getRadians());
    }

    private boolean isDrivingTowardsGamePiece(Translation2d gamePieceFieldRelativePosition, Translation2d robotPosition) {
        final Translation2d fieldRelativeJoystickPosition = getFieldRelativeJoystickPosition();
        final Translation2d fieldRelativeDistanceFromGamePiece = gamePieceFieldRelativePosition.minus(robotPosition);

        final Rotation2d joystickAngle = fieldRelativeJoystickPosition.getAngle();
        final Rotation2d fieldRelativeGamePieceAngleFromRobot = fieldRelativeDistanceFromGamePiece.getAngle();
        final Rotation2d angularOffset = joystickAngle.minus(fieldRelativeGamePieceAngleFromRobot);

        final double velocityTowardsGamePiece = angularOffset.getCos() * fieldRelativeJoystickPosition.getNorm();
        return velocityTowardsGamePiece > OperatorConstants.MINIMUM_VELOCITY_FOR_INTAKE_ASSIST_METERS_PER_SECOND;
    }

    private Translation2d getSelfRelativeJoystickPosition() {
        final double
                joystickX = OperatorConstants.DRIVER_CONTROLLER.getLeftX(),
                joystickY = OperatorConstants.DRIVER_CONTROLLER.getLeftY();

        return new Translation2d(joystickX, joystickY).rotateBy(RobotContainer.SWERVE.getDriveRelativeAngle().unaryMinus());//¯\_(ツ)_/¯
    }

    private Translation2d getFieldRelativeJoystickPosition() {
        final double
                joystickX = OperatorConstants.DRIVER_CONTROLLER.getLeftX(),
                joystickY = OperatorConstants.DRIVER_CONTROLLER.getLeftY();

        return new Translation2d(joystickX, joystickY).rotateBy(Flippable.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);//¯\_(ツ)_/¯
    }

    /**
     * An object representing a single axis of normal intake assist.
     * Normal intake assist adds the joystick input to the assist input after scaling down the assist by the {@code AssistScalar}.
     *
     * @param assistScalar the value with which to scale the assist power (0-1)
     */
    public record NormalAssistAxis(double assistScalar) implements AssistAxis {
        @Override
        public double calculatePower(double pidOutput, double joystickValue, double joystickNorm) {
            if (assistScalar == 0)
                return joystickValue;
            return (joystickValue * (1 - assistScalar)) + (pidOutput * assistScalar);
        }
    }

    /**
     * An object representing a single axis of alternate intake assist.
     * Alternate intake assist scales down the assist power by the joystick's offset from the center.
     * It then adds the joystick value to produce the final output power.
     */
    public record AlternateAssistAxis() implements AssistAxis {
        @Override
        public double calculatePower(double pidOutput, double joystickValue, double joystickNorm) {
            return pidOutput * (1 - Math.cbrt(Math.abs(joystickNorm))) + joystickValue;
        }
    }

    private interface AssistAxis {
        double calculatePower(double pidOutput, double joystickValue, double joystickNorm);
    }
}