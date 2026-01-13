package frc.trigon.robot.subsystems.flyWheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.subsystems.flywheel.SimpleMotorSubsystem;
import frc.trigon.lib.subsystems.flywheel.SimpleMotorSubsystemConfiguration;

public class FlyWheelConstants {
    private static final int
            MASTER_MOTOR_ID = 9,
            FOLLOWER_MOTOR_ID = 10;
    private static final String
            MASTER_MOTOR_NAME = "FlyWheelMasterMotor",
            FOLLOWER_MOTOR_NAME = "FlyWheelFollowerMotor";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);

    private static final String NAME = "FlyWheel";
    private final static double
            GEAR_RATIO = 1,
            MOMENT_OF_INERTIA = 0.003,
            MAXIMUM_VELOCITY = 20,
            MAXIMUM_ACCELERATION = 7,
            MAXIMUM_JERK = 1,
            MAXIMUM_DISPLAYABLE_VELOCITY = 1,
            VELOCITY_TO_TOLERANCE = 1,
            SYS_ID_RAMP_RATE = 3,
            SYS_ID_STEP_VOLTAGE = 1;
    private final static boolean FOC_ENABLED = true;
    private final static boolean SHOULD_USE_VOLTAGE_CONTROL = true;
    private final static DCMotor GEAR_BOX = DCMotor.getKrakenX60Foc(2);

    static SimpleMotorSubsystemConfiguration FLY_WHEEL_CONFIG = new SimpleMotorSubsystemConfiguration();

    static {
        configureSubsystem();
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureSubsystem() {
        FLY_WHEEL_CONFIG.name = NAME;
        FLY_WHEEL_CONFIG.gearRatio = GEAR_RATIO;
        FLY_WHEEL_CONFIG.momentOfInertia = MOMENT_OF_INERTIA;
        FLY_WHEEL_CONFIG.maximumVelocity = MAXIMUM_VELOCITY;
        FLY_WHEEL_CONFIG.maximumAcceleration = MAXIMUM_ACCELERATION;
        FLY_WHEEL_CONFIG.maximumJerk = MAXIMUM_JERK;
        FLY_WHEEL_CONFIG.maximumDisplayableVelocity = MAXIMUM_DISPLAYABLE_VELOCITY;
        FLY_WHEEL_CONFIG.velocityTolerance = VELOCITY_TO_TOLERANCE;
        FLY_WHEEL_CONFIG.sysIDRampRate = SYS_ID_RAMP_RATE;
        FLY_WHEEL_CONFIG.sysIDStepVoltage = SYS_ID_STEP_VOLTAGE;
        FLY_WHEEL_CONFIG.focEnabled = FOC_ENABLED;
        FLY_WHEEL_CONFIG.shouldUseVoltageControl = SHOULD_USE_VOLTAGE_CONTROL;
        FLY_WHEEL_CONFIG.gearbox = GEAR_BOX;
    }

    private static void configureMasterMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;
        config.Slot0.kG = 0;
        config.Slot0.kA = 0;
        config.Slot0.kV = 0;

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

    public enum FlyWheelState implements SimpleMotorSubsystem.SimpleMotorState {
        REST(0, null),
        INTAKE(6, null);

        private final double targetVoltage;
        private final FlyWheelState prepareState;

        FlyWheelState(double targetVoltage, FlyWheelState prepareState) {
            this.targetVoltage = targetVoltage;
            this.prepareState = prepareState;
        }

        @Override
        public double getTargetUnit() {
            return targetVoltage;
        }

        @Override
        public SimpleMotorSubsystem.SimpleMotorState getPrepareState() {
            return prepareState;
        }
    }
}