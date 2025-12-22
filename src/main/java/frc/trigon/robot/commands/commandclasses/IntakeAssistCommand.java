package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import lib.hardware.RobotHardwareStats;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.function.Supplier;

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
                getTrackGamePieceCommand(),
                GeneralCommands.getContinuousConditionalCommand(
                        getAssistIntakeCommand(xAssist, yAssist, thetaAssist, () -> distanceFromTrackedGamePiece),
                        GeneralCommands.getFieldRelativeDriveCommand(),
                        () -> RobotContainer.OBJECT_POSE_ESTIMATOR.hasObject() && distanceFromTrackedGamePiece != null
                )
        );
    }

    /**
     * Returns a command that assists the intake of a game piece at the given location.
     * This version of the command is meant for use when there is a specific target game piece.
     * This command is for intaking a game piece with a specific robot-relative position.
     * To create an intake assist command that selects the closest game piece automatically, use {@link IntakeAssistCommand#IntakeAssistCommand(AssistAxis, AssistAxis, AssistAxis)} instead.
     *
     * @param xAssist                      the assist to apply to the X-axis (Forward and backwards)
     * @param yAssist                      the assist to apply to the Y-axis (Sideways)
     * @param thetaAssist                  the assist to apply to the rotation
     * @param distanceFromTrackedGamePiece the position of the game piece relative to the robot
     * @return the command
     */
    public static Command getAssistIntakeCommand(AssistAxis xAssist, AssistAxis yAssist, AssistAxis thetaAssist, Supplier<Translation2d> distanceFromTrackedGamePiece) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> resetPIDControllers(distanceFromTrackedGamePiece.get())),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> calculateTranslationPower(true, xAssist, distanceFromTrackedGamePiece.get()),
                        () -> calculateTranslationPower(false, yAssist, distanceFromTrackedGamePiece.get()),
                        () -> calculateThetaPower(thetaAssist, distanceFromTrackedGamePiece.get())
                )
        );
    }

    private Command getTrackGamePieceCommand() {
        return new RunCommand(() ->
                distanceFromTrackedGamePiece = RobotContainer.OBJECT_POSE_ESTIMATOR.hasObject() ?
                        calculateDistanceFromBestGamePiece() :
                        null
        );
    }

    private static double calculateTranslationPower(boolean isXAxis, AssistAxis assistAxis, Translation2d distanceFromTrackedGamePiece) {
        final Translation2d selfRelativeJoystickPower = getJoystickPosition().rotateBy(RobotContainer.SWERVE.getDriveRelativeAngle().unaryMinus());
        final double
                joystickPower = isXAxis ? selfRelativeJoystickPower.getX() : selfRelativeJoystickPower.getY(),
                joystickNorm = selfRelativeJoystickPower.getNorm();

        final double assistScalar = assistAxis.getAssistScalar();
        if (assistScalar == 0)
            return joystickPower;

        final double
                pidOutput = clampToOutputRange(isXAxis ?
                X_PID_CONTROLLER.calculate(distanceFromTrackedGamePiece.getX()) :
                Y_PID_CONTROLLER.calculate(distanceFromTrackedGamePiece.getY()));

        if (assistScalar == AlternateAssistAxis.ALTERNATE_ASSIST_SCALAR_VALUE)
            return calculateAlternateAssistTranslationPower(joystickPower, joystickNorm, pidOutput);
        return calculateNormalAssistTranslationPower(assistScalar, joystickPower, pidOutput);
    }

    private static double calculateThetaPower(AssistAxis assistAxis, Translation2d distanceFromTrackedGamePiece) {
        return calculateThetaAssistPower(assistAxis, distanceFromTrackedGamePiece.getAngle().plus(Rotation2d.k180deg).unaryMinus());
    }

    private static double calculateAlternateAssistTranslationPower(double joystickValue, double joystickNorm, double pidOutput) {
        final double
                xJoystickPower = Math.cbrt(joystickValue),
                pidScalar = Math.cbrt(joystickNorm);

        return calculateAlternateAssistPower(pidOutput, pidScalar, xJoystickPower);
    }

    private static double calculateNormalAssistTranslationPower(double assistScalar, double joystickValue, double pidOutput) {
        return calculateNormalAssistPower(assistScalar, joystickValue, pidOutput);
    }

    private static double calculateThetaAssistPower(AssistAxis assistAxis, Rotation2d thetaOffset) {
        final double
                pidOutput = clampToOutputRange(THETA_PID_CONTROLLER.calculate(thetaOffset.getRadians())),
                joystickValue = OperatorConstants.DRIVER_CONTROLLER.getRightX();

        final double assistScalar = assistAxis.getAssistScalar();
        if (assistScalar == 0)
            return joystickValue;

        if (assistScalar == AlternateAssistAxis.ALTERNATE_ASSIST_SCALAR_VALUE)
            return calculateAlternateAssistPower(pidOutput, joystickValue, joystickValue);
        return calculateNormalAssistPower(assistScalar, joystickValue, pidOutput);
    }

    private static double calculateAlternateAssistPower(double pidOutput, double pidScalar, double joystickPower) {
        return pidOutput * (1 - Math.abs(pidScalar)) + joystickPower;
    }

    private static double calculateNormalAssistPower(double intakeAssistScalar, double joystickPower, double pidOutput) {
        return (joystickPower * (1 - intakeAssistScalar)) + (pidOutput * intakeAssistScalar);
    }

    private static double clampToOutputRange(double value) {
        return MathUtil.clamp(value, -1, 1);
    }

    private static void resetPIDControllers(Translation2d distanceFromTrackedGamePiece) {
        X_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getX(), RobotContainer.SWERVE.getSelfRelativeVelocity().vxMetersPerSecond);
        Y_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getY(), RobotContainer.SWERVE.getSelfRelativeVelocity().vyMetersPerSecond);
        THETA_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getAngle().getRadians(), RobotContainer.SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond);
    }

    private static Translation2d calculateDistanceFromBestGamePiece() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final ArrayList<Translation2d> objectPositionsOnField = RobotContainer.OBJECT_POSE_ESTIMATOR.getObjectsOnField();

        Translation2d bestObjectPosition = null;
        for (Translation2d currentObjectPosition : objectPositionsOnField) {
            if (!isGamePieceAssistable(robotPose, currentObjectPosition))
                continue;
            final double currentObjectDistanceFromRobotMeters = currentObjectPosition.getDistance(robotPose.getTranslation());
            if (bestObjectPosition == null || currentObjectDistanceFromRobotMeters < bestObjectPosition.getDistance(robotPose.getTranslation()))
                bestObjectPosition = currentObjectPosition;
        }

        if (bestObjectPosition == null)
            return null;

        final Translation2d robotToBestGamePiece = robotPose.getTranslation().minus(bestObjectPosition);
        final Translation2d robotToBestGamePieceDistance = robotToBestGamePiece.rotateBy(robotPose.getRotation().unaryMinus());
        Logger.recordOutput("IntakeAssist/TargetGamePieceDistance", robotToBestGamePieceDistance);
        return robotToBestGamePieceDistance;
    }

    private static boolean isGamePieceAssistable(Pose2d robotPose, Translation2d gamePiecePosition) {
        final Rotation2d joystickAngle = getJoystickPosition().getAngle().rotateBy(RobotContainer.SWERVE.getDriveRelativeAngle());//Probably bad unit addition
        final double gamePieceDistance = robotPose.getTranslation().getDistance(gamePiecePosition);
        final Rotation2d scaledMaximumAssistAngle = OperatorConstants.INTAKE_ASSIST_MAXIMUM_ASSIST_ANGLE.times(1 / gamePieceDistance);

        return Math.abs(joystickAngle.getRadians()) < Math.abs(scaledMaximumAssistAngle.getRadians());
    }

    private static Translation2d getJoystickPosition() {
        final double
                joystickX = OperatorConstants.DRIVER_CONTROLLER.getLeftX(),
                joystickY = OperatorConstants.DRIVER_CONTROLLER.getLeftY();

        return new Translation2d(joystickX, joystickY);
    }

    /**
     * An object representing a single axis of normal intake assist.
     * Normal intake assist adds the joystick input to the assist input after scaling down the assist by the {@code AssistScalar}.
     *
     * @param assistScalar the value with which to scale the assist power (0-1)
     */
    public record NormalAssistAxis(double assistScalar) implements AssistAxis {
        @Override
        public double getAssistScalar() {
            return assistScalar;
        }
    }

    /**
     * An object representing a single axis of alternate intake assist.
     * Alternate intake assist scales down the assist power by the joystick's offset from the center.
     * It then adds the joystick value to produce the final output power.
     */
    public record AlternateAssistAxis() implements AssistAxis {
        public static double ALTERNATE_ASSIST_SCALAR_VALUE = -1;

        @Override
        public double getAssistScalar() {
            return ALTERNATE_ASSIST_SCALAR_VALUE;
        }
    }

    interface AssistAxis {
        double getAssistScalar();
    }
}