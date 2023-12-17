package frc.trigon.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public abstract class AbstractSubsystem extends edu.wpi.first.wpilibj2.command.SubsystemBase {
    private static final List<AbstractSubsystem> REGISTERED_SUBSYSTEMS = new ArrayList<>();

    public AbstractSubsystem() {
        REGISTERED_SUBSYSTEMS.add(this);
    }

    /**
     * Runs the given consumer on all the subsystem instances.
     *
     * @param toRun the consumer to run on each registered subsystem
     */
    public static void forEach(Consumer<AbstractSubsystem> toRun) {
        REGISTERED_SUBSYSTEMS.forEach(toRun);
    }

    /**
     * Sets whether the subsystem's motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     */
    public abstract void setBrake(boolean brake);

    public abstract void stop();
}
