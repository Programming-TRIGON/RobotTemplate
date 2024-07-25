package frc.trigon.robot.hardware.phoenix6;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.constants.RobotConstants;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Queue;

public class Phoenix6Inputs implements LoggableInputs {
    private final HashMap<String, Queue<Double>> signalToThreadedQueue = new HashMap<>();
    private final Phoenix6SignalThread signalThread = Phoenix6SignalThread.getInstance();
    private final String name;
    private LogTable latestTable = null;
    private double lastErrorTimestamp = 0;
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];

    public Phoenix6Inputs(String name) {
        this.name = name;
    }

    @Override
    public void toLog(LogTable table) {
        if (signals.length == 0)
            return;
        BaseStatusSignal.refreshAll(signals);
        for (BaseStatusSignal signal : signals)
            table.put(signal.getName(), signal.getValueAsDouble());
        for (Map.Entry<String, Queue<Double>> entry : signalToThreadedQueue.entrySet()) {
            table.put(entry.getKey(), entry.getValue().stream().mapToDouble(Double::doubleValue).toArray());
            entry.getValue().clear();
        }
        latestTable = table;
    }

    @Override
    public void fromLog(LogTable table) {
        latestTable = table;
    }

    public void registerSignal(BaseStatusSignal statusSignal, double updateFrequencyHertz) {
        if (statusSignal == null || RobotConstants.IS_REPLAY)
            return;

        statusSignal.setUpdateFrequency(updateFrequencyHertz);
        final BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
        System.arraycopy(signals, 0, newSignals, 0, signals.length);
        newSignals[signals.length] = statusSignal;
        signals = newSignals;
    }

    public void registerThreadedSignal(BaseStatusSignal targetStatusSignal, BaseStatusSignal slopeStatusSignal, double updateFrequencyHertz) {
        if (targetStatusSignal == null || slopeStatusSignal == null || RobotConstants.IS_REPLAY)
            return;

        registerSignal(targetStatusSignal, updateFrequencyHertz);
        registerSignal(slopeStatusSignal, updateFrequencyHertz);
        signalToThreadedQueue.put(targetStatusSignal.getName() + "_Threaded", signalThread.registerSignal(targetStatusSignal, slopeStatusSignal));
    }

    public double getSignal(String signalName) {
        if (latestTable == null) {
            if (shouldPrintError())
                new NullPointerException("The device \"" + name + "\" is trying to retrieve signal \"" + signalName + "\". Though, the latest table is null. This is likely due to the device not being logged.").printStackTrace();
            return 0;
        }

        final LogTable.LogValue value = latestTable.get(signalName);
        if (value == null) {
            if (shouldPrintError())
                new NoSuchElementException("The device \"" + name + "\" is trying to retrieve signal \"" + signalName + "\" which doesn't exist.").printStackTrace();
            return 0;
        }

        return value.getDouble();
    }

    public double[] getThreadedSignal(String signalName) {
        if (latestTable == null) {
            if (shouldPrintError())
                new NullPointerException("The device \"" + name + "\" is trying to retrieve threaded signal \"" + signalName + "\". Though, the latest table is null. This is likely due to the device not being logged.").printStackTrace();
            return new double[0];
        }

        final LogTable.LogValue value = latestTable.get(signalName + "_Threaded");
        if (value == null) {
            if (shouldPrintError())
                new NoSuchElementException("The device \"" + name + "\" is trying to retrieve threaded signal \"" + signalName + "\" which doesn't exist.").printStackTrace();
            return new double[0];
        }

        return value.getDoubleArray();
    }

    private boolean shouldPrintError() {
        final double currentTime = Timer.getFPGATimestamp();
        final boolean shouldPrint = currentTime - lastErrorTimestamp > 10;
        if (shouldPrint)
            lastErrorTimestamp = currentTime;
        return shouldPrint;
    }
}
