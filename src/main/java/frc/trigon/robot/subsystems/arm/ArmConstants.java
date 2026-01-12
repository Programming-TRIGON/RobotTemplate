package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.subsystems.arm.ArmSubsystem;
import frc.trigon.lib.subsystems.arm.ArmSubsystemConfiguration;

public class ArmConstants {
    private static final int
            MASTER_MOTOR_ID = 11,
            FOLLOWER_MOTOR_ID = 12,
            ENCODER_ID = 11;
    private static final String
            MASTER_MOTOR_NAME = "ArmMasterMotor",
            FOLLOWER_MOTOR_NAME = "ArmFollowerMotor",
            ENCODER_NAME = "ArmEncoder";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);
    private static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME);

    private static final String NAME = "Arm";
    private final static double
            GEAR_RATIO = 40,
            MAXIMUM_VELOCITY = 20,
            MAXIMUM_ACCELERATION = 7,
            MAXIMUM_JERK = 1,
            ARM_LENGTH_METERS = 1,
            ARM_MASS_KILOGRAMS = 10,
            SYS_ID_RAMP_RATE = 1,
            SYS_ID_STEP_VOLTAGE = 5,
            VISUALIZATION_OFFSET = 0;
    private final static Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(360),
            ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);
    private final static boolean FOC_ENABLED = true;
    private final static boolean SHOULD_SIMULATE_GRAVITY = true;
    private final static DCMotor GEAR_BOX = DCMotor.getKrakenX60Foc(2);

    static ArmSubsystemConfiguration ARM_CONFIG = new ArmSubsystemConfiguration();

    static {
        configureSubsystem();
        configureMasterMotor();
        configureFollowerMotor();
        configureAngleEncoder();
    }

    private static void configureSubsystem() {
        ARM_CONFIG.name = NAME;
        ARM_CONFIG.gearRatio = GEAR_RATIO;
        ARM_CONFIG.maximumVelocity = MAXIMUM_VELOCITY;
        ARM_CONFIG.maximumAcceleration = MAXIMUM_ACCELERATION;
        ARM_CONFIG.maximumJerk = MAXIMUM_JERK;
        ARM_CONFIG.lengthMeters = ARM_LENGTH_METERS;
        ARM_CONFIG.massKilograms = ARM_MASS_KILOGRAMS;
        ARM_CONFIG.minimumAngle = MINIMUM_ANGLE;
        ARM_CONFIG.maximumAngle = MAXIMUM_ANGLE;
        ARM_CONFIG.angleTolerance = ANGLE_TOLERANCE;
        ARM_CONFIG.shouldSimulateGravity = SHOULD_SIMULATE_GRAVITY;
        ARM_CONFIG.sysIDRampRate = SYS_ID_RAMP_RATE;
        ARM_CONFIG.sysIDStepVoltage = SYS_ID_STEP_VOLTAGE;
        ARM_CONFIG.focEnabled = FOC_ENABLED;
        ARM_CONFIG.gearbox = GEAR_BOX;
        ARM_CONFIG.visualizationOffset = VISUALIZATION_OFFSET;
    }

    private static void configureMasterMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ENCODER.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = 10;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0.01529;
        config.Slot0.kG = 0.78412;
        config.Slot0.kA = 0;
        config.Slot0.kV = 4.9117;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureFollowerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower FollowerRequest = new Follower(MASTER_MOTOR_ID, false);
        FOLLOWER_MOTOR.setControl(FollowerRequest);
    }

    private static void configureAngleEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = VISUALIZATION_OFFSET;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MASTER_MOTOR);

        ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    public enum ArmState implements ArmSubsystem.ArmState {
        REST(Rotation2d.fromDegrees(0), null, 1),
        NINETY(Rotation2d.fromDegrees(90), null, 1),
        ONE_EIGHTY(Rotation2d.fromDegrees(180), null, 1);

        private final Rotation2d targetAngle;
        private final ArmState prepareState;
        private final double speedScalar;

        ArmState(Rotation2d targetAngle, ArmState prepareState, double speedScalar) {
            this.targetAngle = targetAngle;
            this.prepareState = prepareState;
            this.speedScalar = speedScalar;
        }

        @Override
        public Rotation2d getTargetAngle() {
            return targetAngle;
        }

        @Override
        public ArmSubsystem.ArmState getPrepareState() {
            return prepareState;
        }

        @Override
        public double getSpeedScalar() {
            return speedScalar;
        }
    }
}