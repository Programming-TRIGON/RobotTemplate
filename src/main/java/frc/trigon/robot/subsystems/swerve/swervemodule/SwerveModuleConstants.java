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
import frc.trigon.robot.constants.PathPlannerConstants;
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
    private static final double STEER_ENCODER_DISCONTINUITY_POINT = 0.5;
    private static final NeutralModeValue
            DRIVE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake,
            STEER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double
            DRIVE_MOTOR_SLIP_CURRENT = PathPlannerConstants.ROBOT_CONFIG.moduleConfig.driveCurrentLimit, // TODO: calibrate right before competition
            STEER_MOTOR_CURRENT_LIMIT = RobotHardwareStats.isSimulation() ? 200 : 30;
    private static final double
            STEER_MOTOR_P = RobotHardwareStats.isSimulation() ? 120 : 0,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final double
            DRIVE_MOTOR_P = RobotHardwareStats.isSimulation() ? 50 : 50,
            DRIVE_MOTOR_I = 0,
            DRIVE_MOTOR_D = 0,
            DRIVE_MOTOR_KS = RobotHardwareStats.isSimulation() ? 0.4708 : 0,
            DRIVE_MOTOR_KV = RobotHardwareStats.isSimulation() ? 0 : 0,
            DRIVE_MOTOR_KA = RobotHardwareStats.isSimulation() ? 0.48818 : 0;
    static final boolean ENABLE_FOC = true;
    static final double WHEEL_DIAMETER_METERS = PathPlannerConstants.ROBOT_CONFIG.moduleConfig.wheelRadiusMeters * 2;
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

    public static final double MAXIMUM_MODULE_ROTATIONAL_SPEED_RADIANS_PER_SECOND = edu.wpi.first.math.util.Units.rotationsToRadians(10); //TODO: calibrate
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    public static final SysIdRoutine.Config DRIVE_MOTOR_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second),
            Units.Volts.of(3),
            Units.Second.of(1000)
    );

    /**
     * Creates a new SimpleMotorSimulation for the drive motor.
     * We use a function instead of a constant because we need to create a new instance of the simulation for each module.
     *
     * @return the drive motor simulation
     */
    static SimpleMotorSimulation createDriveMotorSimulation() {
        return new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_MOTOR_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA);
    }

    /**
     * Creates a new SimpleMotorSimulation for the steer motor.
     * We use a function instead of a constant because we need to create a new instance of the simulation for each module.
     *
     * @return the steer motor simulation
     */
    static SimpleMotorSimulation createSteerMotorSimulation() {
        return new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, STEER_MOTOR_GEAR_RATIO, STEER_MOMENT_OF_INERTIA);
    }

    private static TalonFXConfiguration generateDriveMotorConfiguration() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = DRIVE_MOTOR_NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = DRIVE_MOTOR_GEAR_RATIO;

        config.TorqueCurrent.PeakForwardTorqueCurrent = DRIVE_MOTOR_SLIP_CURRENT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -DRIVE_MOTOR_SLIP_CURRENT;
        config.CurrentLimits.StatorCurrentLimit = DRIVE_MOTOR_SLIP_CURRENT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = DRIVE_MOTOR_CLOSED_LOOP_RAMP_RATE;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = DRIVE_MOTOR_OPEN_LOOP_RAMP_RATE;

        config.Slot0.kP = DRIVE_MOTOR_P;
        config.Slot0.kI = DRIVE_MOTOR_I;
        config.Slot0.kD = DRIVE_MOTOR_D;
        config.Slot0.kS = DRIVE_MOTOR_KS;
        config.Slot0.kV = DRIVE_MOTOR_KV;
        config.Slot0.kA = DRIVE_MOTOR_KA;

        return config;
    }

    private static TalonFXConfiguration generateSteerMotorConfiguration() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = STEER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = STEER_MOTOR_NEUTRAL_MODE_VALUE;

        config.CurrentLimits.StatorCurrentLimit = STEER_MOTOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Feedback.RotorToSensorRatio = STEER_MOTOR_GEAR_RATIO;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.Slot0.kP = STEER_MOTOR_P;
        config.Slot0.kI = STEER_MOTOR_I;
        config.Slot0.kD = STEER_MOTOR_D;
        config.ClosedLoopGeneral.ContinuousWrap = true;

        return config;
    }

    private static CANcoderConfiguration generateSteerEncoderConfiguration() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = STEER_ENCODER_DISCONTINUITY_POINT;
        config.MagnetSensor.SensorDirection = STEER_ENCODER_DIRECTION;

        return config;
    }
}