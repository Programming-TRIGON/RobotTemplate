package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.InitExecuteCommand;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;

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
    public static Command getClosedLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<MirrorableRotation2d> angleSupplier) {
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
    public static Command getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<MirrorableRotation2d> angleSupplier) {
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
    public static Command getClosedLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<MirrorableRotation2d> angleSupplier) {
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
}