package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.subsystems.elevator.ElevatorSubsystem;
import frc.trigon.lib.subsystems.elevator.ElevatorSubsystemConfiguration;

public class ElevatorConstants {
    private static final int
            MASTER_MOTOR_ID = 13,
            FOLLOWER_MOTOR_ID = 14;
    private static final String
            MASTER_MOTOR_NAME = "ElevatorMasterMotor",
            FOLLOWER_MOTOR_NAME = "ElevatorFollowerMotor";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);

    private static final String NAME = "Elevator";
    private final static double
            GEAR_RATIO = 4,
            MAXIMUM_VELOCITY = 20,
            MAXIMUM_ACCELERATION = 7,
            MAXIMUM_JERK = 70,
            ELEVATOR_MASS_KILOGRAMS = 5,
            SYS_ID_RAMP_RATE = 0.01,
            SYS_ID_STEP_VOLTAGE = 1,
            MINIMUM_HEIGHT_METERS = 0.1,
            MAXIMUM_HEIGHT_METERS = 1.5,
            HEIGHT_TOLERANCE_METERS = 0.1,
            DRUM_RADIUS_METERS = 0.05;
    private final static boolean FOC_ENABLED = true;
    private final static boolean SHOULD_SIMULATE_GRAVITY = true;
    private final static DCMotor GEAR_BOX = DCMotor.getKrakenX60Foc(2);

    static ElevatorSubsystemConfiguration ELEVATOR_CONFIG = new ElevatorSubsystemConfiguration();

    static {
        configureSubsystem();
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureSubsystem() {
        ELEVATOR_CONFIG.name = NAME;
        ELEVATOR_CONFIG.gearRatio = GEAR_RATIO;
        ELEVATOR_CONFIG.maximumVelocity = MAXIMUM_VELOCITY;
        ELEVATOR_CONFIG.maximumAcceleration = MAXIMUM_ACCELERATION;
        ELEVATOR_CONFIG.maximumJerk = MAXIMUM_JERK;
        ELEVATOR_CONFIG.massKilograms = ELEVATOR_MASS_KILOGRAMS;
        ELEVATOR_CONFIG.minimumHeight = MINIMUM_HEIGHT_METERS;
        ELEVATOR_CONFIG.maximumHeight = MAXIMUM_HEIGHT_METERS;
        ELEVATOR_CONFIG.positionToleranceMeters = HEIGHT_TOLERANCE_METERS;
        ELEVATOR_CONFIG.shouldSimulateGravity = SHOULD_SIMULATE_GRAVITY;
        ELEVATOR_CONFIG.sysIDRampRate = SYS_ID_RAMP_RATE;
        ELEVATOR_CONFIG.sysIDStepVoltage = SYS_ID_STEP_VOLTAGE;
        ELEVATOR_CONFIG.focEnabled = FOC_ENABLED;
        ELEVATOR_CONFIG.gearbox = GEAR_BOX;
        ELEVATOR_CONFIG.drumRadiusMeters = DRUM_RADIUS_METERS;
    }

    private static void configureMasterMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = 5;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0.027174;
        config.Slot0.kG = 3;
        config.Slot0.kA = 0;
        config.Slot0.kV = 5;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
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

    public enum ElevatorState implements ElevatorSubsystem.ElevatorState {
        REST(0.2, null, 1),
        HALF(0.7, null, 1),
        FULL(1.5, null, 1);

        private final double targetPosition;
        private final ElevatorState prepareState;
        private final double speedScalar;

        ElevatorState(double targetPosition, ElevatorState prepareState, double speedScalar) {
            this.targetPosition = targetPosition;
            this.prepareState = prepareState;
            this.speedScalar = speedScalar;
        }

        @Override
        public double getTargetPositionMeters() {
            return 0;
        }

        @Override
        public ElevatorSubsystem.ElevatorState getPrepareState() {
            return prepareState;
        }

        @Override
        public double getSpeedScalar() {
            return speedScalar;
        }
    }
}