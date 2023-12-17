package frc.trigon.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

public abstract class AbstractSubsystem extends edu.wpi.first.wpilibj2.command.SubsystemBase {
    public static final List<AbstractSubsystem> REGISTERED_SUBSYSTEMS = new ArrayList<>();

    public AbstractSubsystem() {
        REGISTERED_SUBSYSTEMS.add(this);
    }

    /**
     * Sets whether the subsystem's motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     */
    public abstract void setBrake(boolean brake);

    public abstract void stop();
}
