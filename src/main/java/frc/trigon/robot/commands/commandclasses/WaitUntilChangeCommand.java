package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class WaitUntilChangeCommand<T> extends Command {
    private final Supplier<T> valueSupplier;
    private T lastValue;

    public WaitUntilChangeCommand(Supplier<T> valueSupplier) {
        this.valueSupplier = valueSupplier;
    }

    @Override
    public void initialize() {
        lastValue = valueSupplier.get();
    }

    @Override
    public boolean isFinished() {
        final T currentValue = valueSupplier.get();

        if (currentValue.equals(lastValue)) {
            lastValue = currentValue;
            return false;
        }

        return true;
    }
}
