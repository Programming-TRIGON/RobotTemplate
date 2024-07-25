package frc.trigon.robot.subsystems.swerve.swervemoduleconstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.MotorSimulationPhysicalProperties;

public class SimulationSwerveModuleConstants extends SwerveModuleConstants {
    private static final double WHEEL_DIAMETER_METERS = 0.1016;

    private static final double
            DRIVE_MOMENT_OF_INERTIA = 0.003,
            STEER_MOMENT_OF_INERTIA = 0.003;
    private static final int
            DRIVE_MOTOR_AMOUNT = 1,
            STEER_MOTOR_AMOUNT = 1;
    private static final DCMotor
            DRIVE_MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(DRIVE_MOTOR_AMOUNT),
            STEER_MOTOR_GEARBOX = DCMotor.getFalcon500Foc(STEER_MOTOR_AMOUNT);
    public static final MotorSimulationPhysicalProperties
            DRIVE_PROPERTIES = MotorSimulationPhysicalProperties.createSimpleMotorProperties(DRIVE_MOTOR_GEARBOX, SwerveModuleConstants.DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            STEER_PROPERTIES = MotorSimulationPhysicalProperties.createSimpleMotorProperties(STEER_MOTOR_GEARBOX, SwerveModuleConstants.STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA);

    private static final double
            STEER_MOTOR_P = 75,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;

    public static final TalonFXConfiguration
            DRIVE_CONFIGURATION = generateDriveConfiguration(),
            STEER_CONFIGURATION = generateSteerConfiguration();
    public static final CANcoderConfiguration STEER_ENCODER_CONFIGURATION = generateSteerEncoderConfiguration();

    private static TalonFXConfiguration generateDriveConfiguration() {
        return new TalonFXConfiguration();
    }

    private static TalonFXConfiguration generateSteerConfiguration() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = STEER_MOTOR_P;
        config.Slot0.kI = STEER_MOTOR_I;
        config.Slot0.kD = STEER_MOTOR_D;
        config.ClosedLoopGeneral.ContinuousWrap = true;

        return config;
    }

    private static CANcoderConfiguration generateSteerEncoderConfiguration() {
        return new CANcoderConfiguration();
    }

    @Override
    public double getWheelDiameterMeters() {
        return WHEEL_DIAMETER_METERS;
    }
}
