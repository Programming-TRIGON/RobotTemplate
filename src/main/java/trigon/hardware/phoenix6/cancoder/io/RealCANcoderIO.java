package trigon.hardware.phoenix6.cancoder.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import trigon.hardware.phoenix6.cancoder.CANcoderIO;

public class RealCANcoderIO extends CANcoderIO {
    private final CANcoder cancoder;

    public RealCANcoderIO(int id, String canbus) {
        this.cancoder = new CANcoder(id, canbus);
    }

    @Override
    public void applyConfiguration(CANcoderConfiguration configuration) {
        cancoder.getConfigurator().apply(configuration);
    }

    @Override
    public void optimizeBusUsage() {
        cancoder.optimizeBusUtilization();
    }

    @Override
    protected void setPosition(double positionRotations) {
        cancoder.setPosition(positionRotations);
    }

    @Override
    public CANcoder getCANcoder() {
        return cancoder;
    }
}
