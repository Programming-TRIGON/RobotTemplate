package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.lib.commands.InitExecuteCommand;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {
    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getClosedLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(true),
                () -> RobotContainer.SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed loop mode.
     * This command will use pid to reach the target angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param angleSupplier the target angle supplier
     * @return the command
     */
    public static Command getClosedLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<FlippableRotation2d> angleSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(true),
                () -> RobotContainer.SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(false),
                () -> RobotContainer.SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in open loop mode.
     * This command will use pid to reach the target angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param angleSupplier the target angle supplier
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<FlippableRotation2d> angleSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(false),
                () -> RobotContainer.SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getClosedLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(true),
                () -> RobotContainer.SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's frame of reference, in closed loop mode.
     * This command will use pid to reach the target angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param angleSupplier the target angle supplier
     * @return the command
     */
    public static Command getClosedLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<FlippableRotation2d> angleSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(true),
                () -> RobotContainer.SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get()),
                RobotContainer.SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's frame of reference, in open loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getOpenLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> RobotContainer.SWERVE.initializeDrive(false),
                () -> RobotContainer.SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                RobotContainer.SWERVE
        );
    }

    public static Command getDriveToPoseCommand(Supplier<FlippablePose2d> targetPose, PathConstraints constraints) {
        return new DeferredCommand(() -> getCurrentDriveToPoseCommand(targetPose.get(), constraints), Set.of(RobotContainer.SWERVE));
    }

    public static Command getDriveToPoseCommand(Supplier<FlippablePose2d> targetPose, PathConstraints constraints, double endVelocity) {
        return new DeferredCommand(() -> getCurrentDriveToPoseCommand(targetPose.get(), constraints, endVelocity), Set.of(RobotContainer.SWERVE));
    }

    private static Command getCurrentDriveToPoseCommand(FlippablePose2d targetPose, PathConstraints constraints) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RobotContainer.SWERVE.initializeDrive(true)),
                AutoBuilder.pathfindToPose(targetPose.get(), constraints),
                getPIDToPoseCommand(targetPose)
        );
    }

    private static Command getCurrentDriveToPoseCommand(FlippablePose2d targetPose, PathConstraints constraints, double endVelocity) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RobotContainer.SWERVE.initializeDrive(true)),
                AutoBuilder.pathfindToPose(targetPose.get(), constraints, endVelocity)
        );
    }

    private static Command getPIDToPoseCommand(FlippablePose2d targetPose) {
        return new FunctionalCommand(
                RobotContainer.SWERVE::resetRotationController,
                () -> RobotContainer.SWERVE.pidToPose(targetPose),
                (interrupted) -> {
                },
                () -> RobotContainer.SWERVE.atPose(targetPose)
        );
    }
}