package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * A command class to assist in the intaking of a game piece.
 */
public class IntakeAssistCommand extends ParallelCommandGroup {
    private static final ProfiledPIDController
            X_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(5, 15)) :
            new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(2.65, 5.5)),
            Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
                    new ProfiledPIDController(3, 0, 0.01, new TrapezoidProfile.Constraints(5, 15)) :
                    new ProfiledPIDController(0.3, 0, 0.03, new TrapezoidProfile.Constraints(2.65, 5.5)),
            THETA_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
                    new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
                    new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(2.65, 5.5));
    private Translation2d distanceFromTrackedGamePiece;

    /**
     * Creates a new intake assist command that assists the axes the user inputs.
     *
     * @param xAssist     the assist to apply to the X-axis (Forward and backwards)
     * @param yAssist     the assist to apply to the Y-axis (Sideways)
     * @param thetaAssist the assist to apply to the rotation
     */
    public IntakeAssistCommand(double xAssist, double yAssist, double thetaAssist) {
        addCommands(
                new RunCommand(this::trackGamePiece),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> distanceFromTrackedGamePiece != null),
                        new InstantCommand(this::resetPIDControllers),
                        new WaitUntilCommand(this::hasNoTrackedGamePiece)
                ).repeatedly(),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> calculateTranslationPower(true, xAssist),
                        () -> calculateTranslationPower(false, yAssist),
                        () -> calculateThetaPower(thetaAssist)
                )
        );
    }

    private void resetPIDControllers() {
        X_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getX(), RobotContainer.SWERVE.getSelfRelativeVelocity().getX());
        Y_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getY(), RobotContainer.SWERVE.getSelfRelativeVelocity().getY());
        THETA_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getAngle().getRadians(), RobotContainer.SWERVE.getRotationalVelocityRadiansPerSecond());
    }

    private double calculateTranslationPower(boolean isXAxis, double assistScalar) {
        final Translation2d selfRelativeJoystickValue = getSelfRelativeJoystickPosition();
        final double joystickValue = isXAxis ? selfRelativeJoystickValue.getX() : selfRelativeJoystickValue.getY();

        if (hasNoTrackedGamePiece())
            return joystickValue;

        final double
                pidOutput = clampToSwerveOutputRange(isXAxis ?
                X_PID_CONTROLLER.calculate(distanceFromTrackedGamePiece.getX()) :
                Y_PID_CONTROLLER.calculate(distanceFromTrackedGamePiece.getY()));

        return calculateAssistPower(assistScalar, pidOutput, joystickValue);
    }

    private double calculateThetaPower(double assistScalar) {
        final double joystickValue = OperatorConstants.DRIVER_CONTROLLER.getRightX();

        if (hasNoTrackedGamePiece())
            return joystickValue;

        return calculateThetaAssistPower(assistScalar, joystickValue, distanceFromTrackedGamePiece.getAngle().plus(Rotation2d.k180deg).unaryMinus());
    }

    private double calculateThetaAssistPower(double assistScalar, double joystickValue, Rotation2d thetaOffset) {
        final double pidOutput = clampToSwerveOutputRange(THETA_PID_CONTROLLER.calculate(thetaOffset.getRadians()));
        return calculateAssistPower(assistScalar, pidOutput, joystickValue);
    }

    private double calculateAssistPower(double assistScalar, double pidOutput, double joystickValue) {
        if (assistScalar == 0)
            return joystickValue;
        return (joystickValue * (1 - assistScalar)) + (pidOutput * assistScalar);
    }

    private boolean hasNoTrackedGamePiece() {
        return distanceFromTrackedGamePiece == null;
    }

    private void trackGamePiece() {
        distanceFromTrackedGamePiece = RobotContainer.OBJECT_POSE_ESTIMATOR.hasObject() ?
                calculateDistanceFromBestGamePiece() :
                null;
    }

    private Translation2d calculateDistanceFromBestGamePiece() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d bestGamePieceFieldRelativePosition = getBestGamePieceFieldRelativePosition(robotPose);

        if (bestGamePieceFieldRelativePosition == null)
            return null;

        final Translation2d distanceFromBestGamePiece = robotPose.getTranslation().minus(bestGamePieceFieldRelativePosition);
        final Translation2d robotToBestGamePiece = distanceFromBestGamePiece.rotateBy(robotPose.getRotation().unaryMinus());
        Logger.recordOutput("IntakeAssist/RobotToTargetGamePiece", robotToBestGamePiece);
        return robotToBestGamePiece;
    }

    private Translation2d getBestGamePieceFieldRelativePosition(Pose2d robotPose) {
        return getClosestGamePiece(robotPose.getTranslation(), getAssistableGamePieces(robotPose));
    }

    private ArrayList<Translation2d> getAssistableGamePieces(Pose2d robotPose) {
        final ArrayList<Translation2d> gamePiecePositionsOnField = new ArrayList<>(RobotContainer.OBJECT_POSE_ESTIMATOR.getObjectsOnField());
        gamePiecePositionsOnField.removeIf(
                gamePiecePosition ->
                        !isGamePieceWithinAssistFOV(robotPose, gamePiecePosition) ||
                                !isDrivingTowardsGamePiece(gamePiecePosition, robotPose.getTranslation())
        );

        return gamePiecePositionsOnField;
    }

    private boolean isGamePieceWithinAssistFOV(Pose2d robotPose, Translation2d gamePiecePosition) {
        final Translation2d distanceFromGamePiece = gamePiecePosition.minus(robotPose.getTranslation());
        final Rotation2d robotToGamePieceAngle = distanceFromGamePiece.rotateBy(robotPose.getRotation().unaryMinus()).getAngle();

        final double scaledMaximumAssistAngleDegrees = calculateMaximumAssistAngleDegrees(distanceFromGamePiece.getNorm());

        final Rotation2d angleOffset = getSelfRelativeJoystickPosition().getAngle().minus(robotToGamePieceAngle);
        return Math.abs(angleOffset.getDegrees()) < scaledMaximumAssistAngleDegrees;
    }

    private boolean isDrivingTowardsGamePiece(Translation2d gamePieceFieldRelativePosition, Translation2d robotPosition) {
        final Translation2d fieldRelativeDistanceFromGamePiece = gamePieceFieldRelativePosition.minus(robotPosition);

        final double velocityTowardsGamePiece = getVelocityTowardsGamePiece(fieldRelativeDistanceFromGamePiece);
        return velocityTowardsGamePiece > OperatorConstants.MINIMUM_VELOCITY_FOR_INTAKE_ASSIST_METERS_PER_SECOND;
    }

    private double calculateMaximumAssistAngleDegrees(double gamePieceDistance) {
        return OperatorConstants.INTAKE_ASSIST_MAXIMUM_ASSISTABLE_ANGLE_FORMULA.applyAsDouble(gamePieceDistance);
    }

    private double getVelocityTowardsGamePiece(Translation2d fieldRelativeDistanceFromGamePiece) {
        final Translation2d fieldRelativeJoystickPosition = getFieldRelativeJoystickPosition();

        final Rotation2d joystickAngle = fieldRelativeJoystickPosition.getAngle();
        final Rotation2d fieldRelativeGamePieceAngleFromRobot = fieldRelativeDistanceFromGamePiece.getAngle();
        final Rotation2d angularOffset = joystickAngle.minus(fieldRelativeGamePieceAngleFromRobot);

        final double desiredRobotVelocity = fieldRelativeJoystickPosition.getNorm() * SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND;

        return angularOffset.getCos() * desiredRobotVelocity;
    }

    private Translation2d getClosestGamePiece(Translation2d robotPosition, ArrayList<Translation2d> gamePiecePositionsOnField) {
        Translation2d closestGamePieceFieldRelativePosition = null;
        double distanceFromClosestGamePiece = Double.POSITIVE_INFINITY;

        for (Translation2d gamePieceFieldRelativePosition : gamePiecePositionsOnField) {
            final double distanceFromCurrentGamePiece = gamePieceFieldRelativePosition.getDistance(robotPosition);
            if (distanceFromCurrentGamePiece < distanceFromClosestGamePiece) {
                closestGamePieceFieldRelativePosition = gamePieceFieldRelativePosition;
                distanceFromClosestGamePiece = distanceFromCurrentGamePiece;
            }
        }

        return closestGamePieceFieldRelativePosition;
    }

    private double clampToSwerveOutputRange(double value) {
        return MathUtil.clamp(value, -1, 1);
    }

    private Translation2d getSelfRelativeJoystickPosition() {
        final double
                joystickX = OperatorConstants.DRIVER_CONTROLLER.getLeftX(),
                joystickY = OperatorConstants.DRIVER_CONTROLLER.getLeftY();

        return new Translation2d(joystickY, joystickX).rotateBy(RobotContainer.SWERVE.getDriveRelativeAngle().unaryMinus());
    }

    private Translation2d getFieldRelativeJoystickPosition() {
        final double
                joystickX = OperatorConstants.DRIVER_CONTROLLER.getLeftX(),
                joystickY = OperatorConstants.DRIVER_CONTROLLER.getLeftY();

        return new Translation2d(joystickY, joystickX).rotateBy(Flippable.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
    }
}