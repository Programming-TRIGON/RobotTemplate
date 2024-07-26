package frc.trigon.robot.hardware.phoenix6.cancoder;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.phoenix6.Phoenix6Inputs;
import frc.trigon.robot.hardware.phoenix6.cancoder.io.RealCANcoderIO;
import frc.trigon.robot.hardware.phoenix6.cancoder.io.SimulationCANcoderIO;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class CANcoderEncoder {
    private final String encoderName;
    private final CANcoderIO encoderIO;
    private final Phoenix6Inputs encoderInputs;
    private final int id;

    public CANcoderEncoder(int id, String encoderName, TalonFXMotor simulationMotor) {
        this(id, encoderName, simulationMotor, "");
    }

    public CANcoderEncoder(int id, String encoderName, TalonFXMotor simulationMotor, String canbus) {
        this(id, encoderName, () -> simulationMotor.getSignal(TalonFXSignal.POSITION), () -> simulationMotor.getSignal(TalonFXSignal.VELOCITY), canbus);
    }

    public CANcoderEncoder(int id, String encoderName) {
        this(id, encoderName, null, null, "");
    }

    public CANcoderEncoder(int id, String encoderName, String canbus) {
        this(id, encoderName, null, null, canbus);
    }

    public CANcoderEncoder(int id, String encoderName, DoubleSupplier positionSupplierRotations, DoubleSupplier velocitySupplierRotationsPerSecond) {
        this(id, encoderName, positionSupplierRotations, velocitySupplierRotationsPerSecond, "");
    }

    public CANcoderEncoder(int id, String encoderName, DoubleSupplier positionSupplierRotations, DoubleSupplier velocitySupplierRotationsPerSecond, String canbus) {
        this.encoderName = encoderName;
        this.encoderIO = generateIO(id, positionSupplierRotations, velocitySupplierRotationsPerSecond, canbus);
        this.encoderInputs = new Phoenix6Inputs(encoderName);
        this.id = id;
        encoderIO.optimizeBusUsage();
    }

    public void update() {
        encoderIO.updateEncoder();
        Logger.processInputs("Encoders/" + encoderName, encoderInputs);
    }

    public int getID() {
        return id;
    }

    public void applyConfigurations(CANcoderConfiguration realConfiguration, CANcoderConfiguration simulationConfiguration) {
        if (RobotConstants.IS_SIMULATION)
            encoderIO.applyConfiguration(simulationConfiguration);
        else
            encoderIO.applyConfiguration(realConfiguration);
    }

    public void applyConfiguration(CANcoderConfiguration simulationAndRealConfiguration) {
        encoderIO.applyConfiguration(simulationAndRealConfiguration);
    }

    public void applyRealConfiguration(CANcoderConfiguration realConfiguration) {
        if (!RobotConstants.IS_SIMULATION)
            encoderIO.applyConfiguration(realConfiguration);
    }

    public void applySimulationConfiguration(CANcoderConfiguration simulationConfiguration) {
        if (RobotConstants.IS_SIMULATION)
            encoderIO.applyConfiguration(simulationConfiguration);
    }

    public double getSignal(CANcoderSignal signal) {
        return encoderInputs.getSignal(signal.name);
    }

    public double[] getThreadedSignal(CANcoderSignal signal) {
        return encoderInputs.getThreadedSignal(signal.name);
    }

    public void registerSignal(CANcoderSignal signal, double updateFrequencyHertz) {
        encoderInputs.registerSignal(encoderSignalToStatusSignal(signal), updateFrequencyHertz);
    }

    public void registerThreadedSignal(CANcoderSignal signal, CANcoderSignal slopeSignal, double updateFrequencyHertz) {
        encoderInputs.registerThreadedSignal(encoderSignalToStatusSignal(signal), encoderSignalToStatusSignal(slopeSignal), updateFrequencyHertz);
    }

    private BaseStatusSignal encoderSignalToStatusSignal(CANcoderSignal signal) {
        final CANcoder cancoder = encoderIO.getCANcoder();
        if (RobotConstants.IS_REPLAY || cancoder == null)
            return null;

        return signal.signalFunction.apply(cancoder);
    }

    private CANcoderIO generateIO(int id, DoubleSupplier positionSupplierRotations, DoubleSupplier velocitySupplierRotationsPerSecond, String canbus) {
        if (RobotConstants.IS_REPLAY)
            return new CANcoderIO();
        if (RobotConstants.IS_SIMULATION)
            return new SimulationCANcoderIO(id, positionSupplierRotations, velocitySupplierRotationsPerSecond);
        return new RealCANcoderIO(id, canbus);
    }
}