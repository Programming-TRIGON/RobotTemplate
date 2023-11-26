package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIO {
    private final SwerveModuleInputsAutoLogged swerveModuleInputs = new SwerveModuleInputsAutoLogged();
    private final String name;
    private boolean driveMotorClosedLoop = false;
    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModuleIO(String name) {
        this.name = name;
    }

    /**
     * This method should be called periodically to update the inputs and network tables of the module.
     */
    public void periodic() {
        updateInputs(swerveModuleInputs);
        Logger.processInputs(getLoggingPath(), swerveModuleInputs);
    }

    public void setDriveMotorClosedLoop(boolean closedLoop) {
        driveMotorClosedLoop = closedLoop;
    }

    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = SwerveModuleState.optimize(targetState, getCurrentAngle());
        setTargetAngle(this.targetState.angle);
        setTargetVelocity(this.targetState.speedMetersPerSecond, this.targetState.angle);
    }

    protected String getLoggingPath() {
        return "Swerve/" + name + "/";
    }

    SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(swerveModuleInputs.driveDistanceMeters, getCurrentAngle());
    }

    SwerveModuleState getCurrentState() {
        return new SwerveModuleState(swerveModuleInputs.driveVelocityMetersPerSecond, getCurrentAngle());
    }

    SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Sets the target velocity for the module. In meters per second.
     *
     * @param velocity         the target velocity
     * @param targetSteerAngle the target steer angle, to calculate for skew reduction
     */
    private void setTargetVelocity(double velocity, Rotation2d targetSteerAngle) {
        velocity = reduceSkew(velocity, targetSteerAngle);

        if (driveMotorClosedLoop)
            setTargetClosedLoopVelocity(velocity);
        else
            setTargetOpenLoopVelocity(velocity);
    }

    /**
     * When changing direction, the module will skew since the angle motor is not at its target angle.
     * This method will counter that by reducing the target velocity according to the angle motor's error cosine.
     *
     * @param targetVelocityRevolutions the target velocity in revolutions per second
     * @param targetSteerAngle          the target steer angle
     * @return the reduced target velocity in revolutions per second
     */
    private double reduceSkew(double targetVelocityRevolutions, Rotation2d targetSteerAngle) {
        final double closedLoopError = targetSteerAngle.getRadians() - getCurrentAngle().getRadians();
        final double cosineScalar = Math.abs(Math.cos(closedLoopError));
        return targetVelocityRevolutions * cosineScalar;
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(swerveModuleInputs.steerAngleDegrees);
    }

    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
    }

    /**
     * Sets the target open loop velocity.
     *
     * @param velocity the velocity in meters per second
     */
    protected void setTargetOpenLoopVelocity(double velocity) {
    }

    /**
     * Sets the target closed loop velocity.
     *
     * @param velocity the velocity in meters per second
     */
    protected void setTargetClosedLoopVelocity(double velocity) {
    }

    protected void setTargetAngle(Rotation2d angle) {
    }

    protected void stop() {
    }

    /**
     * Sets whether the module's motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     */
    protected void setBrake(boolean brake) {
    }

    @AutoLog
    public static class SwerveModuleInputs {
        public double steerAngleDegrees = 0;

        public double driveVelocityMetersPerSecond = 0;
        public double driveDistanceMeters = 0;
        public double driveCurrent = 0;
    }
}
