package frc.trigon.robot.subsystems.swerve.swervemodule;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.lib.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.lib.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.lib.utilities.Conversions;

public class SwerveModule {
    private final TalonFXMotor
            driveMotor,
            steerMotor;
    private final CANcoderEncoder steerEncoder;
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(SwerveModuleConstants.ENABLE_FOC);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0).withUpdateFreqHz(SwerveModuleConstants.DRIVE_VELOCITY_REQUEST_UPDATE_FREQUENCY_HERTZ).withEnableFOC(SwerveModuleConstants.ENABLE_FOC);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(SwerveModuleConstants.ENABLE_FOC);
    private final double wheelDiameter;
    private boolean shouldDriveMotorUseClosedLoop = true;
    private SwerveModuleState targetState = new SwerveModuleState();
    private double[]
            latestOdometryDrivePositions,
            latestOdometrySteerPositions;

    /**
     * Constructs a new SwerveModule with the given module ID, wheel diameter, and encoder offset.
     *
     * @param moduleID        the ID of the module
     * @param offsetRotations the module's encoder offset in rotations
     * @param wheelDiameter   the diameter of the wheel
     */
    public SwerveModule(int moduleID, double offsetRotations, double wheelDiameter) {
        driveMotor = new TalonFXMotor(moduleID, "Module" + moduleID + "Drive", RobotConstants.CANIVORE_NAME);
        steerMotor = new TalonFXMotor(moduleID + 4, "Module" + moduleID + "Steer", RobotConstants.CANIVORE_NAME);
        steerEncoder = new CANcoderEncoder(moduleID + 4, "Module" + moduleID + "SteerEncoder", RobotConstants.CANIVORE_NAME);
        this.wheelDiameter = wheelDiameter;

        configureHardware(offsetRotations);
    }

    public void setTargetState(SwerveModuleState targetState) {
        if (willOptimize(targetState)) {
            targetState.optimize(getCurrentSteerAngle());
        }

        this.targetState = targetState;
        setTargetSteerAngle(targetState.angle);
        setTargetDriveVelocity(targetState.speedMetersPerSecond);
    }

    public void setBrake(boolean brake) {
        driveMotor.setBrake(brake);
        steerMotor.setBrake(brake);
    }

    public void updateSysIDLog(SysIdRoutineLog log) {
        log.motor("Module" + driveMotor.getID() + "Drive")
                .angularPosition(Units.Rotations.of(driveMotor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(driveMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(driveMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    /**
     * Updates the swerve module. Should be called periodically.
     * This method updates the hardware, and their position variables.
     * We save their positions to a variable instead of getting them directly because their signals update at a higher frequency than the main code loop.
     */
    public void updatePeriodically() {
        driveMotor.update();
        steerMotor.update();
        steerEncoder.update();

        latestOdometryDrivePositions = driveMotor.getThreadedSignal(TalonFXSignal.POSITION);
        latestOdometrySteerPositions = steerMotor.getThreadedSignal(TalonFXSignal.POSITION);
    }

    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    public void shouldDriveMotorUseClosedLoop(boolean shouldDriveMotorUseClosedLoop) {
        this.shouldDriveMotorUseClosedLoop = shouldDriveMotorUseClosedLoop;
    }

    public void setTargetDriveVoltage(double targetVoltage) {
        driveMotor.setControl(driveVoltageRequest.withOutput(targetVoltage));
    }

    public void setTargetSteerAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(driveWheelRotationsToMeters(driveMotor.getSignal(TalonFXSignal.VELOCITY)), getCurrentSteerAngle());
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Gets the position of the drive wheel in meters. We don't use a {@link Rotation2d} because this function returns distance, not rotations.
     *
     * @return the position of the drive wheel in meters
     */
    public double getDriveWheelPositionRadians() {
        return edu.wpi.first.math.util.Units.rotationsToRadians(driveMotor.getSignal(TalonFXSignal.POSITION));
    }

    /**
     * Since the odometry updates faster than the main code thread,
     * the odometry position is calculated with all odometry positions since the last main thread update.
     *
     * @param odometryUpdateIndex the index of the new odometry information since the last main thread update
     * @return the position of the module at the given odometry update index
     */
    public SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
        return new SwerveModulePosition(
                driveWheelRotationsToMeters(latestOdometryDrivePositions[odometryUpdateIndex]),
                Rotation2d.fromRotations(latestOdometrySteerPositions[odometryUpdateIndex])
        );
    }

    private boolean willOptimize(SwerveModuleState state) {
        final Rotation2d angularDelta = state.angle.minus(getCurrentSteerAngle());
        return Math.abs(angularDelta.getRadians()) > Math.PI / 2;
    }

    /**
     * Sets the target drive velocity for the module.
     * The target velocity is set using either closed loop or open loop control, depending on {@link this#shouldDriveMotorUseClosedLoop}.
     *
     * @param targetVelocityMetersPerSecond the target drive velocity, in meters per second
     */
    private void setTargetDriveVelocity(double targetVelocityMetersPerSecond) {
        if (shouldDriveMotorUseClosedLoop) {
            setTargetClosedLoopDriveVelocity(targetVelocityMetersPerSecond);
            return;
        }

        setTargetOpenLoopDriveVelocity(targetVelocityMetersPerSecond);
    }

    private void setTargetClosedLoopDriveVelocity(double targetVelocityMetersPerSecond) {
        final double targetDriveVelocityRotationsPerSecond = metersToDriveWheelRotations(targetVelocityMetersPerSecond);

        driveMotor.setControl(driveVelocityRequest.withVelocity(targetDriveVelocityRotationsPerSecond));
    }

    private void setTargetOpenLoopDriveVelocity(double targetVelocityMetersPerSecond) {
        final double targetDrivePower = targetVelocityMetersPerSecond / SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND;
        final double targetDriveVoltage = Conversions.compensatedPowerToVoltage(targetDrivePower, SwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);
        driveMotor.setControl(driveVoltageRequest.withOutput(targetDriveVoltage));
    }

    private Rotation2d getCurrentSteerAngle() {
        return Rotation2d.fromRotations(steerMotor.getSignal(TalonFXSignal.POSITION));
    }

    /**
     * Converts drive wheel rotations distance to meters.
     *
     * @param rotations the rotations of the drive wheel
     * @return the distance the drive wheel has traveled in meters
     */
    private double driveWheelRotationsToMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, wheelDiameter);
    }

    /**
     * Converts meters to number of rotations from the drive wheel.
     *
     * @param meters the meters to be converted
     * @return the distance the drive wheel has traveled in drive wheel rotations
     */
    private double metersToDriveWheelRotations(double meters) {
        return Conversions.distanceToRotations(meters, wheelDiameter);
    }

    private void configureHardware(double offsetRotations) {
        configureDriveMotor();
        configureSteerMotor();
        configureSteerEncoder(offsetRotations);
    }

    private void configureDriveMotor() {
        driveMotor.applyConfiguration(SwerveModuleConstants.generateDriveMotorConfiguration());
        driveMotor.setPhysicsSimulation(SwerveModuleConstants.createDriveMotorSimulation());

        driveMotor.registerSignal(TalonFXSignal.VELOCITY, 100);
        driveMotor.registerSignal(TalonFXSignal.TORQUE_CURRENT, 100);
        driveMotor.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        driveMotor.registerThreadedSignal(TalonFXSignal.POSITION, PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }

    private void configureSteerMotor() {
        steerMotor.applyConfiguration(SwerveModuleConstants.generateSteerMotorConfiguration(steerMotor.getID()));
        steerMotor.setPhysicsSimulation(SwerveModuleConstants.createSteerMotorSimulation());

        steerMotor.registerSignal(TalonFXSignal.VELOCITY, 100);
        steerMotor.registerSignal(TalonFXSignal.TORQUE_CURRENT, 100);
        steerMotor.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        steerMotor.registerThreadedSignal(TalonFXSignal.POSITION, PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }

    private void configureSteerEncoder(double offsetRotations) {
        steerEncoder.applyConfiguration(SwerveModuleConstants.generateSteerEncoderConfiguration(offsetRotations));
        steerEncoder.setSimulationInputsFromTalonFX(steerMotor);

        steerEncoder.registerSignal(CANcoderSignal.POSITION, 100);
        steerEncoder.registerSignal(CANcoderSignal.VELOCITY, 100);
    }
}