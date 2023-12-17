package frc.trigon.robot.utilities;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Notifier;

import java.util.HashMap;

/**
 * A thread that sets the brake mode of a TalonFX asynchronously.
 * <p>
 * This is useful for when you want to set the brake mode of a TalonFX, but don't want to wait for the TalonFX to respond, as updating the brake mode performs blocking.
 */
public class TalonFXBrakeThread {
    private static final HashMap<TalonFX, NeutralModeValue> MOTORS_TO_BRAKE = new HashMap<>();
    private static final Notifier NOTIFIER = new Notifier(TalonFXBrakeThread::updateBrakeForAllMotors);
    private static boolean IS_RUNNING = false;

    /**
     * Sets the brake mode of the given TalonFX asynchronously.
     * Note that this method may take some time to set the brake mode, and will not always set it instantly.
     *
     * @param motor             the motor to set the brake mode of
     * @param neutralModeValue  the brake mode to set
     */
    public static void setBrakeAsynchronous(TalonFX motor, NeutralModeValue neutralModeValue) {
        MOTORS_TO_BRAKE.put(motor, neutralModeValue);

        if (IS_RUNNING)
            return;
        NOTIFIER.startPeriodic(0);
        IS_RUNNING = true;
    }

    private static void updateBrakeForAllMotors() {
        while (!MOTORS_TO_BRAKE.isEmpty()) {
            for (TalonFX motor : MOTORS_TO_BRAKE.keySet()) {
                setBrakeSynchronous(motor, MOTORS_TO_BRAKE.get(motor));
                MOTORS_TO_BRAKE.remove(motor);
            }
        }

        IS_RUNNING = false;
    }

    private static void setBrakeSynchronous(TalonFX motor, NeutralModeValue neutralModeValue) {
        final MotorOutputConfigs config = new MotorOutputConfigs();
        final TalonFXConfigurator configurator = motor.getConfigurator();

        configurator.refresh(config, 10);
        config.NeutralMode = neutralModeValue;
        configurator.apply(config, 10);
    }
}
