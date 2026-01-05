package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;

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
     * Creates a new intake assist command of the given assist type.
     *
     * @param assistMode the type of assistance
     */
    public IntakeAssistCommand(AssistMode assistMode) {
        addCommands(
                getTrackGamePieceCommand(),
                GeneralCommands.getContinuousConditionalCommand(
                        GeneralCommands.getFieldRelativeDriveCommand(),
                        getAssistIntakeCommand(assistMode, () -> distanceFromTrackedGamePiece),
                        () -> RobotContainer.OBJECT_POSE_ESTIMATOR.getClosestObjectToRobot() == null || distanceFromTrackedGamePiece == null
                )
        );
    }

    /**
     * Returns a command that assists the intake of a game piece at the given location.
     * This command is for intaking a game piece with a specific robot-relative position.
     * To create an intake assist command that selects the closest game piece automatically, use {@link IntakeAssistCommand#IntakeAssistCommand(AssistMode)} instead.
     *
     * @param assistMode                   the type of assistance
     * @param distanceFromTrackedGamePiece the position of the game piece relative to the robot
     * @return the command
     */
    public static Command getAssistIntakeCommand(AssistMode assistMode, Supplier<Translation2d> distanceFromTrackedGamePiece) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> resetPIDControllers(distanceFromTrackedGamePiece.get())),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> calculateTranslationPower(assistMode, distanceFromTrackedGamePiece.get()).getX(),
                        () -> calculateTranslationPower(assistMode, distanceFromTrackedGamePiece.get()).getY(),
                        () -> calculateThetaPower(assistMode, distanceFromTrackedGamePiece.get())
                )
        );
    }

    private Command getTrackGamePieceCommand() {
        return new RunCommand(() -> {
            if (RobotContainer.OBJECT_POSE_ESTIMATOR.getClosestObjectToRobot() != null)
                distanceFromTrackedGamePiece = calculateDistanceFromTrackedCGamePiece();
        });
    }

    private static Translation2d calculateDistanceFromTrackedCGamePiece() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d trackedObjectPositionOnField = RobotContainer.OBJECT_POSE_ESTIMATOR.getClosestObjectToRobot();
        if (trackedObjectPositionOnField == null)
            return null;

        final Translation2d difference = robotPose.getTranslation().minus(trackedObjectPositionOnField);
        final Translation2d robotToTrackedGamepieceDistance = difference.rotateBy(robotPose.getRotation().unaryMinus());
        Logger.recordOutput("IntakeAssist/TrackedGamePieceDistance", robotToTrackedGamepieceDistance);
        return robotToTrackedGamepieceDistance;
    }

    private static Translation2d calculateTranslationPower(AssistMode assistMode, Translation2d distanceFromTrackedGamepiece) {
        final Translation2d joystickPower = new Translation2d(OperatorConstants.DRIVER_CONTROLLER.getLeftY(), OperatorConstants.DRIVER_CONTROLLER.getLeftX());
        final Translation2d selfRelativeJoystickPower = joystickPower.rotateBy(RobotContainer.SWERVE.getDriveRelativeAngle().unaryMinus());

        final double xPIDOutput = clampToOutputRange(X_PID_CONTROLLER.calculate(distanceFromTrackedGamepiece.getX()));
        final double yPIDOutput = clampToOutputRange(Y_PID_CONTROLLER.calculate(distanceFromTrackedGamepiece.getY()));

        if (assistMode.equals(AssistMode.ALTERNATE_ASSIST))
            return calculateAlternateAssistTranslationPower(selfRelativeJoystickPower, xPIDOutput, yPIDOutput);
        return calculateNormalAssistTranslationPower(assistMode, selfRelativeJoystickPower, xPIDOutput, yPIDOutput);
    }

    private static double calculateThetaPower(AssistMode assistMode, Translation2d distanceFromTrackedGamepiece) {
        return calculateThetaAssistPower(assistMode, distanceFromTrackedGamepiece.getAngle().plus(Rotation2d.k180deg).unaryMinus());
    }

    private static Translation2d calculateAlternateAssistTranslationPower(Translation2d joystickValue, double xPIDOutput, double yPIDOutput) {
        final double pidScalar = Math.cbrt(joystickValue.getNorm());
        final double
                xJoystickPower = Math.cbrt(joystickValue.getX()),
                yJoystickPower = Math.cbrt(joystickValue.getY());
        final double
                xPower = calculateAlternateAssistPower(xPIDOutput, pidScalar, xJoystickPower),
                yPower = calculateAlternateAssistPower(yPIDOutput, pidScalar, yJoystickPower);

        return new Translation2d(xPower, yPower);
    }

    private static Translation2d calculateNormalAssistTranslationPower(AssistMode assistMode, Translation2d joystickValue, double xPIDOutput, double yPIDOutput) {
        final double
                xJoystickPower = joystickValue.getX(),
                yJoystickPower = joystickValue.getY();
        final double
                xPower = assistMode.shouldAssistX ? calculateNormalAssistPower(xPIDOutput, xJoystickPower) : xJoystickPower,
                yPower = assistMode.shouldAssistY ? calculateNormalAssistPower(yPIDOutput, yJoystickPower) : yJoystickPower;

        return new Translation2d(xPower, yPower);
    }

    private static double calculateThetaAssistPower(AssistMode assistMode, Rotation2d thetaOffset) {
        final double
                pidOutput = clampToOutputRange(THETA_PID_CONTROLLER.calculate(thetaOffset.getRadians())),
                joystickValue = OperatorConstants.DRIVER_CONTROLLER.getRightX();

        if (assistMode.equals(AssistMode.ALTERNATE_ASSIST))
            return calculateAlternateAssistPower(pidOutput, joystickValue, joystickValue);
        return calculateNormalAssistPower(pidOutput, joystickValue);
    }

    private static double clampToOutputRange(double value) {
        return MathUtil.clamp(value, -1, 1);
    }

    private static double calculateAlternateAssistPower(double pidOutput, double pidScalar, double joystickPower) {
        return pidOutput * (1 - Math.abs(pidScalar)) + joystickPower;
    }

    private static double calculateNormalAssistPower(double pidOutput, double joystickPower) {
        final double intakeAssistScalar = OperatorConstants.INTAKE_ASSIST_SCALAR;
        return (pidOutput * intakeAssistScalar) + (joystickPower * (1 - intakeAssistScalar));
    }

    private static void resetPIDControllers(Translation2d distanceFromTrackedGamePiece) {
        X_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getX(), RobotContainer.SWERVE.getSelfRelativeVelocity().getX());
        Y_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getY(), RobotContainer.SWERVE.getSelfRelativeVelocity().getY());
        THETA_PID_CONTROLLER.reset(distanceFromTrackedGamePiece.getAngle().getRadians(), RobotContainer.SWERVE.getRotationalVelocityRadiansPerSecond());
    }

    /**
     * An enum containing different modes in which the command assists the intake of the game piece.
     */
    public enum AssistMode {
        /**
         * An alternate method for assisting the intake where the pid output is scaled down the more input the driver gives.
         */
        ALTERNATE_ASSIST(true, true, true),
        /**
         * Applies pid values to autonomously drive to the game piece, scaled by {@link OperatorConstants#INTAKE_ASSIST_SCALAR} in addition to the driver's inputs
         */
        FULL_ASSIST(true, true, true),
        /**
         * Applies pid values to align to the game piece, scaled by {@link OperatorConstants#INTAKE_ASSIST_SCALAR} in addition to the driver's inputs
         */
        ALIGN_ASSIST(false, true, true),
        /**
         * Applies pid values to face the game piece, scaled by {@link OperatorConstants#INTAKE_ASSIST_SCALAR} in addition to the driver's inputs
         */
        ASSIST_ROTATION(false, false, true);

        final boolean
                shouldAssistX,
                shouldAssistY,
                shouldAssistTheta;

        AssistMode(boolean shouldAssistX, boolean shouldAssistY, boolean shouldAssistTheta) {
            this.shouldAssistX = shouldAssistX;
            this.shouldAssistY = shouldAssistY;
            this.shouldAssistTheta = shouldAssistTheta;
        }
    }
}