package lib.hardware.misc.simplesensor.io;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import lib.hardware.misc.simplesensor.SimpleSensorIO;
import lib.hardware.misc.simplesensor.SimpleSensorInputsAutoLogged;

public class DutyCycleSensorIO extends SimpleSensorIO {
    private final DutyCycle dutyCycle;

    public DutyCycleSensorIO(int channel) {
        dutyCycle = new DutyCycle(new DigitalInput(channel));
    }

    public void updateInputs(SimpleSensorInputsAutoLogged inputs) {
        inputs.value = dutyCycle.getHighTimeNanoseconds();
    }
}