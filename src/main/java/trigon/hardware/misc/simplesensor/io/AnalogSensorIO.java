package trigon.hardware.misc.simplesensor.io;

import edu.wpi.first.wpilibj.AnalogInput;
import trigon.hardware.misc.simplesensor.SimpleSensorIO;
import trigon.hardware.misc.simplesensor.SimpleSensorInputsAutoLogged;

public class AnalogSensorIO extends SimpleSensorIO {
    private final AnalogInput analogInput;

    public AnalogSensorIO(int channel) {
        analogInput = new AnalogInput(channel);
    }

    public void updateInputs(SimpleSensorInputsAutoLogged inputs) {
        inputs.value = analogInput.getValue();
    }
}
