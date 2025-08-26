package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

import java.util.function.BooleanSupplier;

/**
 * A class that contains the general commands of the robot, such as commands that alter a command or commands that affect all subsystems.
 * These are different from {@link CommandConstants} because they create new commands that use some form of logic instead of only constructing an existing command with parameters.
 */
public class GeneralCommands {
    public static Command getFieldRelativeDriveCommand() {
        return SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getRightX())
        );
    }

    public static Command getToggleBrakeCommand() {
        return new InstantCommand(() -> {
            MotorSubsystem.IS_BRAKING = !MotorSubsystem.IS_BRAKING;
            MotorSubsystem.setAllSubsystemsBrakeAsync(MotorSubsystem.IS_BRAKING);
        }).ignoringDisable(true);
    }

    public static Command getDelayedCommand(double delaySeconds, Runnable toRun) {
        return new WaitCommand(delaySeconds).andThen(toRun).ignoringDisable(true);
    }

    public static Command getContinuousConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        return new ConditionalCommand(
                onTrue.onlyWhile(condition),
                onFalse.until(condition),
                condition
        ).repeatedly();
    }

    /**
     * A command that only runs when a condition is met.
     *
     * @param command   the command to run
     * @param condition the condition that needs to be met for the command to run
     * @return the command
     */
    public static Command runWhen(Command command, BooleanSupplier condition) {
        return new WaitUntilCommand(condition).andThen(command);
    }

    /**
     * A command that only runs when a condition is met for a certain amount of time.
     *
     * @param command             the command to run
     * @param condition           the condition that needs to be met for the command to run
     * @param debounceTimeSeconds the time that the condition needs to be true for the command to run
     * @return the command
     */
    public static Command runWhen(Command command, BooleanSupplier condition, double debounceTimeSeconds) {
        return runWhen(new WaitCommand(debounceTimeSeconds).andThen(command.onlyIf(condition)), condition);
    }
}