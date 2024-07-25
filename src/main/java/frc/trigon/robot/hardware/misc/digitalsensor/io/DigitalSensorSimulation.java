package frc.trigon.robot.hardware.misc.digitalsensor.io;

import frc.trigon.robot.hardware.misc.digitalsensor.DigitalSensorIO;
import frc.trigon.robot.hardware.misc.digitalsensor.DigitalSensorInputsAutoLogged;

import java.util.function.BooleanSupplier;

public class DigitalSensorSimulation extends DigitalSensorIO {
    private final BooleanSupplier isTriggered;

    public DigitalSensorSimulation(BooleanSupplier isTriggered) {
        this.isTriggered = isTriggered;
    }

    public void updateInputs(DigitalSensorInputsAutoLogged inputs) {
        inputs.isTriggered = isTriggered.getAsBoolean();
    }
}
