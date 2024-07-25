package frc.trigon.robot.hardware.phoenix6.talonfx.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.trigon.robot.hardware.phoenix6.talonfx.MotorSimulationPhysicalProperties;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXIO;
import frc.trigon.robot.hardware.simulation.*;

public class SimulationTalonFXIO extends TalonFXIO {
    private final MotorPhysicsSimulation motorSimulation;
    private final TalonFX talonFX;
    private final TalonFXSimState motorSimState;

    public SimulationTalonFXIO(int id, MotorSimulationPhysicalProperties physicalProperties) {
        this.motorSimulation = generateMotorSimulation(id, physicalProperties);
        this.talonFX = new TalonFX(id);
        this.motorSimState = talonFX.getSimState();
        motorSimState.setSupplyVoltage(12);
    }

    @Override
    public void updateMotor() {
        motorSimulation.setInputVoltage(motorSimState.getMotorVoltage());
        motorSimulation.updateMotor();
        motorSimState.setRawRotorPosition(motorSimulation.getPositionRevolutions());
        motorSimState.setRotorVelocity(motorSimulation.getVelocityRevolutionsPerSecond());
    }

    @Override
    public void setControl(ControlRequest request) {
        talonFX.setControl(request);
    }

    @Override
    public void applyConfiguration(TalonFXConfiguration configuration) {
        talonFX.getConfigurator().apply(configuration);
    }

    @Override
    public void optimizeBusUsage() {
        talonFX.optimizeBusUtilization();
    }

    @Override
    public void stopMotor() {
        talonFX.stopMotor();
    }

    @Override
    public TalonFX getTalonFX() {
        return talonFX;
    }

    private MotorPhysicsSimulation generateMotorSimulation(int id, MotorSimulationPhysicalProperties physicalProperties) {
        return switch (physicalProperties.simulationType) {
            case ELEVATOR ->
                    new ElevatorSimulation(physicalProperties.gearbox, physicalProperties.gearRatio, physicalProperties.elevatorProperties.carriageMassKilograms, physicalProperties.elevatorProperties.drumRadiusMeters, physicalProperties.elevatorProperties.retractedHeightMeters, physicalProperties.elevatorProperties.maximumHeightMeters, physicalProperties.elevatorProperties.simulateGravity);
            case SIMPLE_MOTOR ->
                    new SimpleMotorSimulation(physicalProperties.gearbox, physicalProperties.gearRatio, physicalProperties.simpleMotorProperties.momentOfInertia);
            case FLYWHEEL ->
                    new FlywheelSimulation(physicalProperties.gearbox, physicalProperties.flywheelProperties.gearRatio, physicalProperties.flywheelProperties.kv, physicalProperties.flywheelProperties.ka);
            case SINGLE_JOINTED_ARM ->
                    new SingleJointedArmSimulation(physicalProperties.gearbox, physicalProperties.gearRatio, physicalProperties.singleJointedArmProperties.armLengthMeters, physicalProperties.singleJointedArmProperties.armMassKilograms, physicalProperties.singleJointedArmProperties.minimumAngle, physicalProperties.singleJointedArmProperties.maximumAngle, physicalProperties.singleJointedArmProperties.simulateGravity);
        };
    }
}
