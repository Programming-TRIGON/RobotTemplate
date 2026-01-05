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
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.robot.constants.AutonomousConstants;

public class SwerveModuleConstants {
    private static final double
            DRIVE_MOTOR_GEAR_RATIO = 7.13,
            REAR_STEER_MOTOR_GEAR_RATIO = 12.8;
    static final boolean ENABLE_FOC = true;

    private static final double
            DRIVE_MOMENT_OF_INERTIA = 0.003,
            STEER_MOMENT_OF_INERTIA = 0.003;
    private static final int
            DRIVE_MOTOR_AMOUNT = 1,
            STEER_MOTOR_AMOUNT = 1;
    private static final DCMotor
            DRIVE_MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(DRIVE_MOTOR_AMOUNT),
            STEER_MOTOR_GEARBOX = DCMotor.getFalcon500Foc(STEER_MOTOR_AMOUNT);

    public static final SysIdRoutine.Config DRIVE_MOTOR_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second),
            Units.Volts.of(2),
            Units.Second.of(1000)
    );

    public static final double MAXIMUM_MODULE_ROTATIONAL_SPEED_RADIANS_PER_SECOND = edu.wpi.first.math.util.Units.rotationsToRadians(7); //TODO: calibrate
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    static final double DRIVE_VELOCITY_REQUEST_UPDATE_FREQUENCY_HERTZ = 1000;

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
        return new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, REAR_STEER_MOTOR_GEAR_RATIO, STEER_MOMENT_OF_INERTIA);
    }

    static TalonFXConfiguration generateDriveMotorConfiguration() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = DRIVE_MOTOR_GEAR_RATIO;

        final double driveMotorSlipCurrent = AutonomousConstants.ROBOT_CONFIG.moduleConfig.driveCurrentLimit;
        config.TorqueCurrent.PeakForwardTorqueCurrent = driveMotorSlipCurrent;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -driveMotorSlipCurrent;
        config.CurrentLimits.StatorCurrentLimit = driveMotorSlipCurrent;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.016046 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0.8774 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 00.020691 : 0;

        config.Feedback.VelocityFilterTimeConstant = 0;

        return config;
    }

    static TalonFXConfiguration generateSteerMotorConfiguration(int feedbackRemoteSensorID) {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.StatorCurrentLimit = RobotHardwareStats.isSimulation() ? 200 : 50;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Feedback.RotorToSensorRatio = REAR_STEER_MOTOR_GEAR_RATIO;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = feedbackRemoteSensorID;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 120 : 40;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.ClosedLoopGeneral.ContinuousWrap = true;

        return config;
    }

    static CANcoderConfiguration generateSteerEncoderConfiguration(double offsetRotations) {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = offsetRotations;

        return config;
    }
}