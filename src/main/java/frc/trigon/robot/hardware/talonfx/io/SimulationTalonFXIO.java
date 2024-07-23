package frc.trigon.robot.hardware.talonfx.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.hardware.simulation.*;
import frc.trigon.robot.hardware.talonfx.MotorSimulationPhysicalProperties;
import frc.trigon.robot.hardware.talonfx.TalonFXIO;

public class SimulationTalonFXIO extends TalonFXIO {
    private final MotorSimulation motorSimulation;

    public SimulationTalonFXIO(int id, MotorSimulationPhysicalProperties physicalProperties) {
        this.motorSimulation = generateMotorSimulation(id, physicalProperties);
    }

    @Override
    public void updateMotor() {
        motorSimulation.updateSimulation();
    }

    @Override
    public void setControl(ControlRequest request) {
        motorSimulation.setControl(request);
    }

    @Override
    public void applyConfiguration(TalonFXConfiguration configuration) {
        motorSimulation.applyConfiguration(configuration);
    }

    @Override
    public void optimizeBusUsage() {
        motorSimulation.getTalonFX().optimizeBusUtilization();
    }

    @Override
    public void stopMotor() {
        motorSimulation.stop();
    }

    @Override
    public TalonFX getTalonFX() {
        return motorSimulation.getTalonFX();
    }

    private MotorSimulation generateMotorSimulation(int id, MotorSimulationPhysicalProperties physicalProperties) {
        return switch (physicalProperties.simulationType) {
            case ELEVATOR ->
                    new ElevatorSimulation(id, physicalProperties.gearbox, physicalProperties.gearRatio, physicalProperties.elevatorProperties.carriageMassKilograms, physicalProperties.elevatorProperties.drumRadiusMeters, physicalProperties.elevatorProperties.retractedHeightMeters, physicalProperties.elevatorProperties.maximumHeightMeters, physicalProperties.elevatorProperties.simulateGravity);
            case SIMPLE_MOTOR ->
                    new SimpleMotorSimulation(id, physicalProperties.gearbox, physicalProperties.gearRatio, physicalProperties.simpleMotorProperties.momentOfInertia);
            case FLYWHEEL ->
                    new FlywheelSimulation(id, physicalProperties.gearbox, physicalProperties.flywheelProperties.gearRatio, physicalProperties.flywheelProperties.kv, physicalProperties.flywheelProperties.ka);
            case SINGLE_JOINTED_ARM ->
                    new SingleJointedArmSimulation(id, physicalProperties.gearbox, physicalProperties.gearRatio, physicalProperties.singleJointedArmProperties.armLengthMeters, physicalProperties.singleJointedArmProperties.armMassKilograms, physicalProperties.singleJointedArmProperties.minimumAngle, physicalProperties.singleJointedArmProperties.maximumAngle, physicalProperties.singleJointedArmProperties.simulateGravity);
        };
    }
}
