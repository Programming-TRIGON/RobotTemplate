package frc.trigon.robot.hardware.phoenix6.talonfx;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class MotorSimulationPhysicalProperties {
    public SimulationType simulationType = SimulationType.SIMPLE_MOTOR;
    public DCMotor gearbox;
    public double gearRatio;
    public SimpleMotorProperties simpleMotorProperties;
    public ElevatorProperties elevatorProperties;
    public FlywheelProperties flywheelProperties;
    public SingleJointedArmProperties singleJointedArmProperties;

    public static MotorSimulationPhysicalProperties createSimpleMotorProperties(DCMotor gearbox, double gearRatio, double momentOfInertia) {
        MotorSimulationPhysicalProperties properties = new MotorSimulationPhysicalProperties();
        properties.simulationType = SimulationType.SIMPLE_MOTOR;
        properties.gearbox = gearbox;
        properties.gearRatio = gearRatio;
        properties.simpleMotorProperties = new SimpleMotorProperties();
        properties.simpleMotorProperties.gearRatio = gearRatio;
        properties.simpleMotorProperties.momentOfInertia = momentOfInertia;
        return properties;
    }

    public static MotorSimulationPhysicalProperties createElevatorProperties(DCMotor gearbox, double gearRatio, double drumRadiusMeters, double retractedHeightMeters, double maximumHeightMeters, double carriageMassKilograms, boolean simulateGravity) {
        MotorSimulationPhysicalProperties properties = new MotorSimulationPhysicalProperties();
        properties.simulationType = SimulationType.ELEVATOR;
        properties.gearbox = gearbox;
        properties.gearRatio = gearRatio;
        properties.elevatorProperties = new ElevatorProperties();
        properties.elevatorProperties.drumRadiusMeters = drumRadiusMeters;
        properties.elevatorProperties.retractedHeightMeters = retractedHeightMeters;
        properties.elevatorProperties.maximumHeightMeters = maximumHeightMeters;
        properties.elevatorProperties.carriageMassKilograms = carriageMassKilograms;
        properties.elevatorProperties.simulateGravity = simulateGravity;
        return properties;
    }

    public static MotorSimulationPhysicalProperties createFlywheelProperties(DCMotor gearbox, double gearRatio, double kv, double ka) {
        MotorSimulationPhysicalProperties properties = new MotorSimulationPhysicalProperties();
        properties.simulationType = SimulationType.FLYWHEEL;
        properties.gearbox = gearbox;
        properties.gearRatio = gearRatio;
        properties.flywheelProperties = new FlywheelProperties();
        properties.flywheelProperties.gearRatio = gearRatio;
        properties.flywheelProperties.kv = kv;
        properties.flywheelProperties.ka = ka;
        return properties;
    }

    public static MotorSimulationPhysicalProperties createSingleJointedArmProperties(DCMotor gearbox, double gearRatio, double armLengthMeters, double armMassKilograms, Rotation2d minimumAngle, Rotation2d maximumAngle, boolean simulateGravity) {
        MotorSimulationPhysicalProperties properties = new MotorSimulationPhysicalProperties();
        properties.simulationType = SimulationType.SINGLE_JOINTED_ARM;
        properties.gearbox = gearbox;
        properties.gearRatio = gearRatio;
        properties.singleJointedArmProperties = new SingleJointedArmProperties();
        properties.singleJointedArmProperties.armLengthMeters = armLengthMeters;
        properties.singleJointedArmProperties.armMassKilograms = armMassKilograms;
        properties.singleJointedArmProperties.minimumAngle = minimumAngle;
        properties.singleJointedArmProperties.maximumAngle = maximumAngle;
        properties.singleJointedArmProperties.simulateGravity = simulateGravity;
        return properties;
    }

    public static class SimpleMotorProperties {
        public double gearRatio;
        public double momentOfInertia;
    }

    public static class FlywheelProperties {
        public double gearRatio;
        public double kv;
        public double ka;
    }

    public static class ElevatorProperties {
        public double drumRadiusMeters;
        public double retractedHeightMeters;
        public double maximumHeightMeters;
        public double carriageMassKilograms;
        public boolean simulateGravity;
    }

    public static class SingleJointedArmProperties {
        public double armLengthMeters;
        public double armMassKilograms;
        public Rotation2d minimumAngle;
        public Rotation2d maximumAngle;
        public boolean simulateGravity;
    }

    public enum SimulationType {
        SINGLE_JOINTED_ARM,
        ELEVATOR,
        SIMPLE_MOTOR,
        FLYWHEEL
    }
}
