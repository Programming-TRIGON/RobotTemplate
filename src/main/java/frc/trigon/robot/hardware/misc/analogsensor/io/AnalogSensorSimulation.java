package frc.trigon.robot.hardware.misc.analogsensor.io;

import frc.trigon.robot.hardware.misc.analogsensor.AnalogSensorIO;
import frc.trigon.robot.hardware.misc.analogsensor.AnalogSensorInputsAutoLogged;

import java.util.function.DoubleSupplier;

public class AnalogSensorSimulation extends AnalogSensorIO {
    private final DoubleSupplier isTriggered;

    public AnalogSensorSimulation(DoubleSupplier isTriggered) {
        this.isTriggered = isTriggered;
    }

    public void updateInputs(AnalogSensorInputsAutoLogged inputs) {
        inputs.value = isTriggered.getAsDouble();
    }
}
