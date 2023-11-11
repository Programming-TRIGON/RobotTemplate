package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Notifier;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.utilities.Conversions;

public class TrihardSwerveModuleConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    static final boolean ENABLE_FOC = true;
    static final double COUPLING_RATIO = 0;
    static final double
            DRIVE_GEAR_RATIO = 5.14,
            STEER_GEAR_RATIO = 12.8;
    static final double WHEEL_DIAMETER_METERS = 0.1016;
    static final double MAX_THEORETICAL_SPEED_METERS_PER_SECOND = 4;

    static final int
            FRONT_LEFT_ID = 0,
            FRONT_RIGHT_ID = 1,
            REAR_LEFT_ID = 2,
            REAR_RIGHT_ID = 3;
    private static final int
            FRONT_LEFT_DRIVE_MOTOR_ID = FRONT_LEFT_ID + 1,
            FRONT_RIGHT_DRIVE_MOTOR_ID = FRONT_RIGHT_ID + 1,
            REAR_LEFT_DRIVE_MOTOR_ID = REAR_LEFT_ID + 1,
            REAR_RIGHT_DRIVE_MOTOR_ID = REAR_RIGHT_ID + 1;
    private static final int
            FRONT_LEFT_STEER_MOTOR_ID = FRONT_LEFT_ID + 5,
            FRONT_RIGHT_STEER_MOTOR_ID = FRONT_RIGHT_ID + 5,
            REAR_LEFT_STEER_MOTOR_ID = REAR_LEFT_ID + 5,
            REAR_RIGHT_STEER_MOTOR_ID = REAR_RIGHT_ID + 5;

    private static final InvertedValue
            DRIVE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            STEER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final double
            DRIVE_OPEN_LOOP_RAMP_RATE = 0.1,
            DRIVE_CLOSED_LOOP_RAMP_RATE = 0.2;

    //TODO: check gains
    private static final double
            STEER_MOTOR_P = 0.656875,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final double
            DRIVE_MOTOR_P = 0,
            DRIVE_MOTOR_I = 0,
            DRIVE_MOTOR_D = 0,
            DRIVE_MOTOR_KS = 0.675593,
            DRIVE_MOTOR_KV = 8.4759406,
            DRIVE_MOTOR_KA = 1.2042201;

    private static final TalonFX
            FRONT_LEFT_DRIVE_MOTOR = new TalonFX(FRONT_LEFT_DRIVE_MOTOR_ID),
            FRONT_RIGHT_DRIVE_MOTOR = new TalonFX(FRONT_RIGHT_DRIVE_MOTOR_ID),
            REAR_LEFT_DRIVE_MOTOR = new TalonFX(REAR_LEFT_DRIVE_MOTOR_ID),
            REAR_RIGHT_DRIVE_MOTOR = new TalonFX(REAR_RIGHT_DRIVE_MOTOR_ID);
    private static final TalonFX
            FRONT_LEFT_STEER_MOTOR = new TalonFX(FRONT_LEFT_STEER_MOTOR_ID),
            FRONT_RIGHT_STEER_MOTOR = new TalonFX(FRONT_RIGHT_STEER_MOTOR_ID),
            REAR_LEFT_STEER_MOTOR = new TalonFX(REAR_LEFT_STEER_MOTOR_ID),
            REAR_RIGHT_STEER_MOTOR = new TalonFX(REAR_RIGHT_STEER_MOTOR_ID);

    private static final double ENCODER_UPDATE_TIME_SECONDS = 3;
    private static final int ENCODER_CHANNEL_OFFSET = 1;
    private static final int
            FRONT_LEFT_ENCODER_CHANNEL = FRONT_LEFT_ID + ENCODER_CHANNEL_OFFSET,
            FRONT_RIGHT_ENCODER_CHANNEL = FRONT_RIGHT_ID + ENCODER_CHANNEL_OFFSET,
            REAR_LEFT_ENCODER_CHANNEL = REAR_LEFT_ID + ENCODER_CHANNEL_OFFSET,
            REAR_RIGHT_ENCODER_CHANNEL = REAR_RIGHT_ID + ENCODER_CHANNEL_OFFSET;
    private static final double
            FRONT_LEFT_ENCODER_OFFSET = Conversions.degreesToRevolutions(311.064148),
            FRONT_RIGHT_ENCODER_OFFSET = Conversions.degreesToRevolutions(299.171448),
            REAR_LEFT_ENCODER_OFFSET = Conversions.degreesToRevolutions(504.691315),
            REAR_RIGHT_ENCODER_OFFSET = Conversions.degreesToRevolutions(-31.997681);
    private static final DutyCycleEncoder
            FRONT_LEFT_ENCODER = new DutyCycleEncoder(FRONT_LEFT_ENCODER_CHANNEL),
            FRONT_RIGHT_ENCODER = new DutyCycleEncoder(FRONT_RIGHT_ENCODER_CHANNEL),
            REAR_LEFT_ENCODER = new DutyCycleEncoder(REAR_LEFT_ENCODER_CHANNEL),
            REAR_RIGHT_ENCODER = new DutyCycleEncoder(REAR_RIGHT_ENCODER_CHANNEL);

    static final TrihardSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
                    FRONT_LEFT_DRIVE_MOTOR,
                    FRONT_LEFT_STEER_MOTOR,
                    FRONT_LEFT_ENCODER,
                    FRONT_LEFT_ENCODER_OFFSET
            ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR,
                    FRONT_RIGHT_ENCODER,
                    FRONT_RIGHT_ENCODER_OFFSET
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR,
                    REAR_LEFT_ENCODER,
                    REAR_LEFT_ENCODER_OFFSET
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR,
                    REAR_RIGHT_ENCODER,
                    REAR_RIGHT_ENCODER_OFFSET
            );

    private final Notifier setSteerMotorPositionToAbsoluteNotifier = new Notifier(this::setSteerMotorPositionToAbsolute);
    final TalonFX driveMotor, steerMotor;
    final DutyCycleEncoder steerEncoder;
    final double encoderOffset;

    public TrihardSwerveModuleConstants(TalonFX driveMotor, TalonFX steerMotor, DutyCycleEncoder steerEncoder, double encoderOffset) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;
        this.encoderOffset = encoderOffset;

        if (!RobotConstants.IS_REPLAY) {
            configureDriveMotor();
            configureSteerMotor();
        }
    }

    private void configureSteerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = STEER_MOTOR_INVERTED_VALUE;
        config.Feedback.SensorToMechanismRatio = STEER_GEAR_RATIO;

        config.Slot0.kP = STEER_MOTOR_P;
        config.Slot0.kI = STEER_MOTOR_I;
        config.Slot0.kD = STEER_MOTOR_D;
        config.ClosedLoopGeneral.ContinuousWrap = true;

        steerMotor.getConfigurator().apply(config);

        steerMotor.getPosition().setUpdateFrequency(150);
        steerMotor.getStatorCurrent().setUpdateFrequency(20);
        steerMotor.getVelocity().setUpdateFrequency(150);
        steerMotor.getClosedLoopError().setUpdateFrequency(70);
        steerMotor.optimizeBusUtilization();

        setSteerMotorPositionToAbsoluteNotifier.startSingle(ENCODER_UPDATE_TIME_SECONDS);
    }

    private void configureDriveMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED_VALUE;
        config.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DRIVE_OPEN_LOOP_RAMP_RATE;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DRIVE_CLOSED_LOOP_RAMP_RATE;

        config.Slot0.kP = DRIVE_MOTOR_P;
        config.Slot0.kI = DRIVE_MOTOR_I;
        config.Slot0.kD = DRIVE_MOTOR_D;
        config.Slot0.kS = DRIVE_MOTOR_KS;
        config.Slot0.kV = DRIVE_MOTOR_KV;
        config.Slot0.kA = DRIVE_MOTOR_KA;

        driveMotor.getConfigurator().apply(config);

        driveMotor.getPosition().setUpdateFrequency(100);
        driveMotor.getVelocity().setUpdateFrequency(100);
        driveMotor.getDutyCycle().setUpdateFrequency(10);
        driveMotor.optimizeBusUtilization();
    }

    private void setSteerMotorPositionToAbsolute() {
        // TODO: Check if this works properly
        final double offsettedRevolutions = Conversions.offsetRead(steerEncoder.getAbsolutePosition(), encoderOffset);
        steerMotor.setPosition(offsettedRevolutions);
        setSteerMotorPositionToAbsoluteNotifier.close();
    }
}
