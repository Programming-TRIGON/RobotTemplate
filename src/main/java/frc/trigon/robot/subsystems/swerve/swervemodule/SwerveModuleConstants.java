package frc.trigon.robot.subsystems.swerve.swervemodule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.simulation.SimpleMotorSimulation;

public class SwerveModuleConstants {
    private static final double
            DRIVE_MOTOR_GEAR_RATIO = 6.12,
            STEER_MOTOR_GEAR_RATIO = 12.8;
    private static final double
            DRIVE_MOTOR_OPEN_LOOP_RAMP_RATE = RobotHardwareStats.isSimulation() ? 0.1 : 0.1,
            DRIVE_MOTOR_CLOSED_LOOP_RAMP_RATE = RobotHardwareStats.isSimulation() ? 0.1 : 0.1;
    private static final InvertedValue
            DRIVE_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            STEER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final SensorDirectionValue STEER_ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final double STEER_ENCODER_RANGE = 0.5;
    private static final NeutralModeValue
            DRIVE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake,
            STEER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double
            DRIVE_MOTOR_SLIP_CURRENT = RobotHardwareStats.isSimulation() ? 1000 : 80, // TODO: calibrate before competition
            STEER_MOTOR_CURRENT_LIMIT = RobotHardwareStats.isSimulation() ? 1000 : 30;
    private static final double
            STEER_MOTOR_P = RobotHardwareStats.isSimulation() ? 75 : 75,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final double
            DRIVE_MOTOR_P = RobotHardwareStats.isSimulation() ? 50 : 50,
            DRIVE_MOTOR_I = 0,
            DRIVE_MOTOR_D = 0,
            DRIVE_MOTOR_KS = RobotHardwareStats.isSimulation() ? 1 : 0,
            DRIVE_MOTOR_KV = RobotHardwareStats.isSimulation() ? 1 : 0,
            DRIVE_MOTOR_KA = RobotHardwareStats.isSimulation() ? 1 : 0;
    static final boolean ENABLE_FOC = true;
    static final TalonFXConfiguration
            DRIVE_MOTOR_CONFIGURATION = generateDriveMotorConfiguration(),
            STEER_MOTOR_CONFIGURATION = generateSteerMotorConfiguration();
    static final CANcoderConfiguration STEER_ENCODER_CONFIGURATION = generateSteerEncoderConfiguration();

    private static final double
            DRIVE_MOMENT_OF_INERTIA = 0.003,
            STEER_MOMENT_OF_INERTIA = 0.003;
    private static final int
            DRIVE_MOTOR_AMOUNT = 1,
            STEER_MOTOR_AMOUNT = 1;
    private static final DCMotor
            DRIVE_MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(DRIVE_MOTOR_AMOUNT),
            STEER_MOTOR_GEARBOX = DCMotor.getFalcon500Foc(STEER_MOTOR_AMOUNT);
    static final SimpleMotorSimulation
            DRIVE_MOTOR_SIMULATION = createDriveMotorSimulation(),
            STEER_MOTOR_SIMULATION = createSteerMotorSimulation();

    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    public static final SysIdRoutine.Config DRIVE_MOTOR_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(5).per(Units.Second),
            Units.Volts.of(8),
            Units.Second.of(1000)
    );

    private static TalonFXConfiguration generateDriveMotorConfiguration() {
        final TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Audio.BeepOnBoot = false;
        configuration.Audio.BeepOnConfig = false;

        configuration.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED_VALUE;
        configuration.MotorOutput.NeutralMode = DRIVE_MOTOR_NEUTRAL_MODE_VALUE;
        configuration.Feedback.SensorToMechanismRatio = DRIVE_MOTOR_GEAR_RATIO;

        configuration.TorqueCurrent.PeakForwardTorqueCurrent = DRIVE_MOTOR_SLIP_CURRENT;
        configuration.TorqueCurrent.PeakReverseTorqueCurrent = -DRIVE_MOTOR_SLIP_CURRENT;
        configuration.CurrentLimits.StatorCurrentLimit = DRIVE_MOTOR_SLIP_CURRENT;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        configuration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = DRIVE_MOTOR_CLOSED_LOOP_RAMP_RATE;
        configuration.OpenLoopRamps.VoltageOpenLoopRampPeriod = DRIVE_MOTOR_OPEN_LOOP_RAMP_RATE;

        configuration.Slot0.kP = DRIVE_MOTOR_P;
        configuration.Slot0.kI = DRIVE_MOTOR_I;
        configuration.Slot0.kD = DRIVE_MOTOR_D;
        configuration.Slot0.kS = DRIVE_MOTOR_KS;
        configuration.Slot0.kV = DRIVE_MOTOR_KV;
        configuration.Slot0.kA = DRIVE_MOTOR_KA;

        return configuration;
    }

    private static TalonFXConfiguration generateSteerMotorConfiguration() {
        final TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Audio.BeepOnBoot = false;
        configuration.Audio.BeepOnConfig = false;

        configuration.MotorOutput.Inverted = STEER_MOTOR_INVERTED_VALUE;
        configuration.MotorOutput.NeutralMode = STEER_MOTOR_NEUTRAL_MODE_VALUE;

        configuration.CurrentLimits.StatorCurrentLimit = STEER_MOTOR_CURRENT_LIMIT;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        configuration.Feedback.RotorToSensorRatio = STEER_MOTOR_GEAR_RATIO;
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        configuration.Slot0.kP = STEER_MOTOR_P;
        configuration.Slot0.kI = STEER_MOTOR_I;
        configuration.Slot0.kD = STEER_MOTOR_D;
        configuration.ClosedLoopGeneral.ContinuousWrap = true;

        return configuration;
    }

    private static CANcoderConfiguration generateSteerEncoderConfiguration() {
        final CANcoderConfiguration configuration = new CANcoderConfiguration();

        configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = STEER_ENCODER_RANGE;
        configuration.MagnetSensor.SensorDirection = STEER_ENCODER_DIRECTION;

        return configuration;
    }

    private static SimpleMotorSimulation createDriveMotorSimulation() {
        return new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_MOTOR_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA);
    }

    private static SimpleMotorSimulation createSteerMotorSimulation() {
        return new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, STEER_MOTOR_GEAR_RATIO, STEER_MOMENT_OF_INERTIA);
    }
}