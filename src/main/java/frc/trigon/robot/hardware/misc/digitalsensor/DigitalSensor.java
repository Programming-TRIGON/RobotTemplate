package frc.trigon.robot.hardware.misc.digitalsensor;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.misc.digitalsensor.io.DigitalSensorSimulation;
import frc.trigon.robot.hardware.misc.digitalsensor.io.PWMDigitalSensorIO;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

public class DigitalSensor {
    private final String name;
    private final DigitalSensorIO sensorIO;
    private final DigitalSensorInputsAutoLogged sensorInputs = new DigitalSensorInputsAutoLogged();

    public DigitalSensor(String name, int channel) {
        this(name, channel, () -> false);
    }

    public DigitalSensor(String name, int channel, BooleanSupplier isTriggeredSimulation) {
        this.name = name;
        this.sensorIO = generateIO(channel, isTriggeredSimulation);
    }

    public boolean isTriggered() {
        return sensorInputs.isTriggered;
    }

    public void updateSensor() {
        sensorIO.updateInputs(sensorInputs);
        Logger.processInputs("DigitalSensors/" + name, sensorInputs);
    }

    private DigitalSensorIO generateIO(int channel, BooleanSupplier isTriggeredSimulation) {
        if (RobotConstants.IS_REPLAY)
            return new DigitalSensorIO();
        if (RobotConstants.IS_SIMULATION)
            return new DigitalSensorSimulation(isTriggeredSimulation);
        return new PWMDigitalSensorIO(channel);
    }
}
