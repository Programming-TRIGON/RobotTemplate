package frc.trigon.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
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
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];

    public Phoenix6Inputs(String name) {
        this.name = name;
    }

    public static String enumNameToSignalName(String enumName) {
        final String lowercaseName = enumName.toLowerCase();
        final String nameNoUnderscore = lowercaseName.replace("_", "");
        final char[] camelCaseNameChars = new char[nameNoUnderscore.length()];

        boolean wasLastUnderscore = false;
        camelCaseNameChars[0] = Character.toUpperCase(lowercaseName.charAt(0));
        int lastIndex = 1;
        for (int i = 1; i < lowercaseName.length(); i++) {
            final char currentChar = lowercaseName.charAt(i);

            if (currentChar == '_') {
                wasLastUnderscore = true;
                continue;
            }

            if (wasLastUnderscore) {
                wasLastUnderscore = false;
                camelCaseNameChars[lastIndex] = Character.toUpperCase(currentChar);
            } else {
                camelCaseNameChars[lastIndex] = currentChar;
            }

            lastIndex++;
        }

        return new String(camelCaseNameChars);
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
        if (latestTable == null)
            return 0;
        final LogTable.LogValue value = latestTable.get(signalName);
        if (value == null)
            throw new NoSuchElementException("The device \"" + name + "\" is to retrieve signal \"" + signalName + "\" which doesn't exist.");
        return value.getDouble();
    }

    public double[] getThreadedSignal(String signalName) {
        if (latestTable == null)
            return new double[0];
        final LogTable.LogValue value = latestTable.get(signalName + "_Threaded");
        if (value == null) {
            throw new NoSuchElementException("The device \"" + name + "\" is to retrieve threaded signal \"" + signalName + "\" which doesn't exist.");
        }
        return value.getDoubleArray();
    }
}
