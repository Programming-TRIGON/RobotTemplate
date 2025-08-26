package lib.hardware.phoenix6.cancoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import lib.utilities.Conversions;

import java.util.function.Function;

/**
 * An enum that represents a signal that can be read from a CANcoder.
 */
public enum CANcoderSignal {
    POSITION(CANcoder::getPosition),
    VELOCITY(CANcoder::getVelocity);

    final String name;
    final Function<CANcoder, BaseStatusSignal> signalFunction;

    CANcoderSignal(Function<CANcoder, BaseStatusSignal> signalFunction) {
        this.name = Conversions.snakeCaseToCamelCase(name());
        this.signalFunction = signalFunction;
    }
}
