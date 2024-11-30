package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;

import java.util.function.BooleanSupplier;

public class GeneralCommands {
    public static boolean IS_BRAKING = true;

    public static Command withoutRequirements(Command command) {
        return new FunctionalCommand(
                command::initialize,
                command::execute,
                command::end,
                command::isFinished
        );
    }

    /**
     * @return a command that toggles between the SWERVE's default command, from field relative to self relative
     */
    public static Command getToggleFieldAndSelfRelativeDriveCommand() {
        return new InstantCommand(() -> {
            if (RobotContainer.SWERVE.getDefaultCommand().equals(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND))
                RobotContainer.SWERVE.setDefaultCommand(CommandConstants.SELF_RELATIVE_DRIVE_COMMAND);
            else
                RobotContainer.SWERVE.setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);

            RobotContainer.SWERVE.getDefaultCommand().schedule();
        });
    }

    public static Command getToggleBrakeCommand() {
        return new InstantCommand(() -> {
            IS_BRAKING = !IS_BRAKING;
            MotorSubsystem.setAllSubsystemsBrakeAsync(IS_BRAKING);

            if (IS_BRAKING)
                CommandConstants.STATIC_WHITE_LED_COLOR_COMMAND.cancel();
            else
                CommandConstants.STATIC_WHITE_LED_COLOR_COMMAND.schedule();
        }).ignoringDisable(true);
    }

    public static Command getDelayedCommand(double delaySeconds, Runnable toRun) {
        return new WaitCommand(delaySeconds).andThen(toRun).ignoringDisable(true);
    }

    public static Command getContinuousConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        return new ConditionalCommand(
                onTrue.onlyWhile(condition),
                onFalse.onlyWhile(() -> !condition.getAsBoolean()),
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
        return new WaitUntilCommand(condition).andThen(new WaitCommand(debounceTimeSeconds).andThen(command.onlyIf(condition)));
    }

    public static Command duplicate(Command command) {
        return new FunctionalCommand(
                command::initialize,
                command::execute,
                command::end,
                command::isFinished,
                command.getRequirements().toArray(Subsystem[]::new)
        );
    }
}
