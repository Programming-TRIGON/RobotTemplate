package frc.trigon.robot.commands.commandclasses.rumblecommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.commands.commandclasses.rumblecommands.rumbleconfigurations.RumbleChain;
import frc.trigon.robot.commands.commandclasses.rumblecommands.rumbleconfigurations.RumbleConfiguration;
import frc.trigon.robot.commands.commandclasses.rumblecommands.rumbleconfigurations.SingleRumble;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RumbleCommands {
    public static Command getRumbleWhenConditionIsMetCommand(RumbleConfiguration rumbleConfiguration, BooleanSupplier condition) {
        return new SequentialCommandGroup(
                new WaitUntilCommand(condition),
                rumbleConfiguration.getRumbleCommand(),
                new WaitUntilCommand(() -> !condition.getAsBoolean())
        ).repeatedly();
    }

    public static Command getRumbleWhenConditionIsMetCommand(RumbleConfiguration rumbleConfiguration, BooleanSupplier condition, double debounceTimeSeconds) {
        final Debouncer debouncer = new Debouncer(debounceTimeSeconds);
        final BooleanSupplier debouncedCondition = () -> debouncer.calculate(condition.getAsBoolean());

        return getRumbleWhenConditionIsMetCommand(rumbleConfiguration, debouncedCondition);
    }

    public static Command getRumbleOncePerSecondCommand(Supplier<RumbleConfiguration> rumbleConfiguration) {
        return getRumbleOncePerPeriodCommand(rumbleConfiguration, () -> 1);
    }

    public static Command getRumbleOncePerPeriodCommand(Supplier<RumbleConfiguration> rumbleConfiguration, DoubleSupplier periodSeconds) {
        return new DeferredCommand(() -> getRumbleAndWaitCommand(rumbleConfiguration, periodSeconds), Set.of()).repeatedly();
    }

    private static Command getRumbleAndWaitCommand(Supplier<RumbleConfiguration> rumbleConfiguration, DoubleSupplier periodSeconds) {
        return new SequentialCommandGroup(
                rumbleConfiguration.get().getRumbleCommand(),
                new WaitCommand(periodSeconds.getAsDouble())
        );
    }

    public enum PremadeRumbles implements RumbleConfiguration {
        BIG(new SingleRumble(0.8, 1)),
        SMALL(new SingleRumble(0.5, 1)),
        TINY(new SingleRumble(0.2, 0.8)),
        DOUBLE_TAP(RumbleChain.constructRumbleChainFromLinks(
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1), 0.08),
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1), 0)
        )),
        TRIPLE_TAP(RumbleChain.constructRumbleChainFromLinks(
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1), 0.08),
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1), 0.08),
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1), 0)
        )),
        LEFT(new SingleRumble(0.5, 1, GenericHID.RumbleType.kLeftRumble)),
        RIGHT(new SingleRumble(0.5, 1, GenericHID.RumbleType.kRightRumble)),
        QUICK_LEFT_THEN_RIGHT(RumbleChain.constructRumbleChainFromLinks(
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1, GenericHID.RumbleType.kLeftRumble), 0),
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1, GenericHID.RumbleType.kRightRumble), 0)
        )),
        QUICK_RIGHT_THEN_LEFT(RumbleChain.constructRumbleChainFromLinks(
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1, GenericHID.RumbleType.kRightRumble), 0),
                new RumbleChain.RumbleChainLink(new SingleRumble(0.2, 1, GenericHID.RumbleType.kLeftRumble), 0)
        )),
        NONE(new SingleRumble(0, 0));

        final RumbleConfiguration rumbleConfiguration;

        PremadeRumbles(RumbleConfiguration rumbleConfiguration) {
            this.rumbleConfiguration = rumbleConfiguration;
        }

        @Override
        public Command getRumbleCommand() {
            return rumbleConfiguration.getRumbleCommand();
        }
    }
}
