package frc.trigon.robot.commands.commandclasses.rumblecommands.rumbleconfigurations;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.constants.OperatorConstants;

public class SingleRumble implements RumbleConfiguration {
    private double durationSeconds, power;
    private GenericHID.RumbleType rumbleMotors;

    public SingleRumble(double durationSeconds, double power, GenericHID.RumbleType rumbleMotors) {
        this.durationSeconds = durationSeconds;
        this.power = power;
        setRumbleMotors(rumbleMotors);
    }

    public SingleRumble(double durationSeconds, double power) {
        this(durationSeconds, power, GenericHID.RumbleType.kBothRumble);
    }

    public Command getRumbleCommand() {
        return new InstantCommand(() -> OperatorConstants.DRIVER_CONTROLLER.rumble(
                durationSeconds,
                power,
                rumbleMotors
        ));
    }

    public double getDurationSeconds() {
        return durationSeconds;
    }

    public double getPower() {
        return power;
    }

    public GenericHID.RumbleType getRumbleMotors() {
        return rumbleMotors;
    }

    public void setDurationSeconds(double durationSeconds) {
        this.durationSeconds = durationSeconds;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setRumbleMotors(GenericHID.RumbleType rumbleMotors) {
        if (rumbleMotors.equals(GenericHID.RumbleType.kLeftRumble))//Left and right motors are flipped on our controller
            rumbleMotors = GenericHID.RumbleType.kRightRumble;
        else if (rumbleMotors.equals(GenericHID.RumbleType.kRightRumble))
            rumbleMotors = GenericHID.RumbleType.kLeftRumble;

        this.rumbleMotors = rumbleMotors;
    }
}
