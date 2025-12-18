package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.swerve.swervemodule.SwerveModule;
import frc.trigon.robot.subsystems.swerve.swervemodule.SwerveModuleConstants;
import lib.hardware.RobotHardwareStats;
import lib.hardware.phoenix6.Phoenix6SignalThread;
import lib.hardware.phoenix6.pigeon2.Pigeon2Gyro;
import lib.hardware.phoenix6.pigeon2.Pigeon2Signal;
import lib.utilities.flippable.Flippable;
import lib.utilities.flippable.FlippablePose2d;
import lib.utilities.flippable.FlippableRotation2d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends MotorSubsystem {
    private final Pigeon2Gyro gyro = SwerveConstants.GYRO;
    private final SwerveModule[] swerveModules = SwerveConstants.SWERVE_MODULES;
    private final Phoenix6SignalThread phoenix6SignalThread = Phoenix6SignalThread.getInstance();
    private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(AutonomousConstants.ROBOT_CONFIG, SwerveModuleConstants.MAXIMUM_MODULE_ROTATIONAL_SPEED_RADIANS_PER_SECOND);
    public Pose2d targetPathPlannerPose = new Pose2d();
    public boolean isPathPlannerDriving = false;
    private SwerveSetpoint previousSetpoint = new SwerveSetpoint(getSelfRelativeVelocity(), getModuleStates(), DriveFeedforwards.zeros(AutonomousConstants.ROBOT_CONFIG.numModules));

    public Swerve() {
        setName("Swerve");
        phoenix6SignalThread.setThreadFrequencyHertz(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }

    @Override
    public void setBrake(boolean brake) {
        for (SwerveModule module : swerveModules)
            module.setBrake(brake);
    }

    @Override
    public void sysIDDrive(double targetCurrent) {
        SwerveModuleState[] a = SwerveConstants.KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, 0, 2));
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDriveMotorTargetCurrent(targetCurrent);
            swerveModules[i].setTargetAngle(a[i].angle);
        }
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        for (SwerveModule module : swerveModules)
            module.updateSysIDLog(log);
    }

    @Override
    public void updatePeriodically() {
        Phoenix6SignalThread.SIGNALS_LOCK.lock();
        updateHardware();
        Phoenix6SignalThread.SIGNALS_LOCK.unlock();

        updatePoseEstimatorStates();
        RobotContainer.ROBOT_POSE_ESTIMATOR.periodic();
//        updateNetworkTables();
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return SwerveModuleConstants.DRIVE_MOTOR_SYSID_CONFIG;
    }

    @Override
    public void stop() {
        for (SwerveModule module : swerveModules)
            module.stop();
    }

    public void setHeading(Rotation2d heading) {
        gyro.setYaw(heading);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(SwerveConstants.GYRO.getSignal(Pigeon2Signal.YAW));
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        final ChassisSpeeds selfRelativeSpeeds = getSelfRelativeVelocity();
        return ChassisSpeeds.fromRobotRelativeSpeeds(selfRelativeSpeeds, RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation());
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return SwerveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /**
     * Checks if the robot is at a pose.
     *
     * @param pose2d a pose, relative to the blue alliance driver station's right corner
     * @return whether the robot is at the pose
     */
    public boolean atPose(FlippablePose2d pose2d) {
        final Pose2d flippedPose = pose2d.get();
        return atXAxisPosition(flippedPose.getX()) && atYAxisPosition(flippedPose.getY()) && atAngle(pose2d.getRotation());
    }

    public boolean atXAxisPosition(double xAxisPosition) {
        final double currentXAxisVelocity = getFieldRelativeVelocity().vxMetersPerSecond;
        return atTranslationPosition(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getX(), xAxisPosition, currentXAxisVelocity);
    }

    public boolean atYAxisPosition(double yAxisPosition) {
        final double currentYAxisVelocity = getFieldRelativeVelocity().vyMetersPerSecond;
        return atTranslationPosition(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getY(), yAxisPosition, currentYAxisVelocity);
    }

    public boolean atAngle(FlippableRotation2d angle) {
        final boolean atTargetAngle = Math.abs(angle.get().minus(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation()).getDegrees()) < SwerveConstants.ROTATION_TOLERANCE_DEGREES;
        final boolean isAngleStill = Math.abs(getSelfRelativeVelocity().omegaRadiansPerSecond) < SwerveConstants.ROTATION_VELOCITY_TOLERANCE;
        Logger.recordOutput("Swerve/AtTargetAngle/isStill", isAngleStill);
        Logger.recordOutput("Swerve/AtTargetAngle/atTargetAngle", atTargetAngle);
        return atTargetAngle;
    }

    /**
     * Gets the positions of the drive wheels in radians. We don't use a {@link Rotation2d} because this method returns distance, not rotations.
     *
     * @return the positions of the drive wheels in radians
     */
    public double[] getDriveWheelPositionsRadians() {
        final double[] swerveModulesPositions = new double[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++)
            swerveModulesPositions[i] = swerveModules[i].getDriveWheelPositionRadians();
        return swerveModulesPositions;
    }

    public void setTargetPathPlannerPose(Pose2d targetPathPlannerPose) {
        this.targetPathPlannerPose = targetPathPlannerPose;
    }

    public void drivePathPlanner(ChassisSpeeds targetPathPlannerFeedforwardSpeeds, boolean isFromPathPlanner) {
        isPathPlannerDriving = !isStill(targetPathPlannerFeedforwardSpeeds);
        if (!isPathPlannerDriving) {
            pidToPose(new FlippablePose2d(targetPathPlannerPose, false));
            return;
        }

        if (isFromPathPlanner && DriverStation.isAutonomous() && !isPathPlannerDriving)
            return;
        final ChassisSpeeds pidSpeeds = calculateSelfRelativePIDToPoseSpeeds(new FlippablePose2d(targetPathPlannerPose, false));
        final ChassisSpeeds scaledSpeeds = targetPathPlannerFeedforwardSpeeds.times(AutonomousConstants.FEEDFORWARD_SCALAR);
        final ChassisSpeeds combinedSpeeds = pidSpeeds.plus(scaledSpeeds);
        selfRelativeDrive(combinedSpeeds);
    }

    /**
     * Drives the swerve with robot-relative targetSpeeds without using setpoint generation.
     * Used for PathPlanner because it generates setpoints automatically.
     *
     * @param targetSpeeds the pre generated robot relative target speeds
     * @param feedforwards the feedforwards to use
     */
    public void selfRelativeDriveWithoutSetpointGeneration(ChassisSpeeds targetSpeeds, DriveFeedforwards feedforwards) {
//        if (isStill(targetSpeeds)) {
//            stop();
//            return;
//        }

        final SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(targetSpeeds);
        final double[] targetForcesNm;
        if (feedforwards == null)
            targetForcesNm = new double[]{0, 0, 0, 0};
        else
            targetForcesNm = feedforwards.linearForcesNewtons();

        setTargetModuleStates(swerveModuleStates, targetForcesNm);
    }

    public Rotation2d getDriveRelativeAngle() {
        final Rotation2d currentAngle = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation();
        return Flippable.isRedAlliance() ? currentAngle.rotateBy(Rotation2d.fromDegrees(180)) : currentAngle;
    }

    public void resetSetpoint() {
        previousSetpoint = new SwerveSetpoint(getSelfRelativeVelocity(), getModuleStates(), DriveFeedforwards.zeros(AutonomousConstants.ROBOT_CONFIG.numModules));
    }

    public void initializeDrive(boolean shouldUseClosedLoop) {
        setClosedLoop(shouldUseClosedLoop);
        resetRotationController();
    }

    void resetRotationController() {
        SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.reset(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation().getDegrees());
    }

    /**
     * Moves the robot to a certain pose using PID.
     *
     * @param targetPose the target pose, relative to the blue alliance driver station's right corner
     */
    void pidToPose(FlippablePose2d targetPose) {
        final ChassisSpeeds targetSpeeds = calculateSelfRelativePIDToPoseSpeeds(targetPose);
        selfRelativeDrive(targetSpeeds);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the field's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle, relative to the blue alliance's forward position
     */
    void fieldRelativeDrive(double xPower, double yPower, FlippableRotation2d targetAngle) {
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the field's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     */
    void fieldRelativeDrive(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the robot's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     */
    void selfRelativeDrive(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the robot's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle
     */
    void selfRelativeDrive(double xPower, double yPower, FlippableRotation2d targetAngle) {
        final ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);
        selfRelativeDrive(speeds);
    }

    /**
     * This method will take in desired robot-relative chassis targetSpeeds,
     * generate a swerve setpoint, then set the target state for each module.
     *
     * @param targetSpeeds the desired robot-relative targetSpeeds
     */
    public void selfRelativeDrive(ChassisSpeeds targetSpeeds) {
        previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint,
                targetSpeeds,
                RobotHardwareStats.getPeriodicTimeSeconds()
        );

//        if (isStill(previousSetpoint.robotRelativeSpeeds())) {
//            stop();
//            return;
//        }

        setTargetModuleStates(previousSetpoint.moduleStates(), previousSetpoint.feedforwards().linearForcesNewtons());
    }

    private void setTargetModuleStates(SwerveModuleState[] swerveModuleStates, double[] targetForcesNm) {
        for (int i = 0; i < swerveModules.length; i++)
            swerveModules[i].setTargetState(swerveModuleStates[i], targetForcesNm[i]);
    }

    private void setClosedLoop(boolean shouldUseClosedLoop) {
        for (SwerveModule currentModule : swerveModules)
            currentModule.shouldDriveMotorUseClosedLoop(shouldUseClosedLoop);
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND,
                yPower * SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND,
                thetaPower * SwerveConstants.MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND
        );
    }

    private ChassisSpeeds calculateSelfRelativePIDToPoseSpeeds(FlippablePose2d targetPose) {
        final Pose2d currentPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Pose2d flippedTargetPose = targetPose.get();
        final double xSpeed = SwerveConstants.X_TRANSLATION_PID_CONTROLLER.calculate(currentPose.getX(), flippedTargetPose.getX());
        final double ySpeed = SwerveConstants.Y_TRANSLATION_PID_CONTROLLER.calculate(currentPose.getY(), flippedTargetPose.getY());
        final int direction = Flippable.isRedAlliance() ? -1 : 1;
        final ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                xSpeed * direction,
                ySpeed * direction,
                calculateProfiledAngleSpeedToTargetAngle(targetPose.getRotation())
        );

        return fieldRelativeSpeedsToSelfRelativeSpeeds(targetFieldRelativeSpeeds);
    }

    private void updatePoseEstimatorStates() {
        final double[] odometryUpdatesYawDegrees = gyro.getThreadedSignal(Pigeon2Signal.YAW);
        final int odometryUpdates = odometryUpdatesYawDegrees.length;
        final SwerveModulePosition[][] swerveWheelPositions = new SwerveModulePosition[odometryUpdates][];
        final Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = getSwerveWheelPositions(i);
            gyroRotations[i] = Rotation2d.fromDegrees(odometryUpdatesYawDegrees[i]);
        }

        RobotContainer.ROBOT_POSE_ESTIMATOR.updatePoseEstimatorOdometry(swerveWheelPositions, gyroRotations, phoenix6SignalThread.getLatestTimestamps());
    }

    private SwerveModulePosition[] getSwerveWheelPositions(int odometryUpdateIndex) {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++)
            swerveModulePositions[i] = swerveModules[i].getOdometryPosition(odometryUpdateIndex);
        return swerveModulePositions;
    }

    private boolean atTranslationPosition(double currentTranslationPosition, double targetTranslationPosition, double currentTranslationVelocity) {
        return Math.abs(currentTranslationPosition - targetTranslationPosition) < SwerveConstants.TRANSLATION_TOLERANCE_METERS &&
                Math.abs(currentTranslationVelocity) < SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    private double calculateProfiledAngleSpeedToTargetAngle(FlippableRotation2d targetAngle) {
        if (targetAngle == null)
            return 0;
        SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.setGoal(targetAngle.get().getDegrees());

        final Rotation2d currentAngle = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation();
        final double output = Units.degreesToRadians(SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.calculate(currentAngle.getDegrees()));
        final boolean atGoal = SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.atGoal();
        Logger.recordOutput("Swerve/ProfiledRotationPIDController/Setpoint", SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.getSetpoint().position);
        Logger.recordOutput("Swerve/ProfiledRotationPIDController/Current", currentAngle.getDegrees());
        Logger.recordOutput("Swerve/ProfiledRotationPIDController/AtGoal", atGoal);
        Logger.recordOutput("Swerve/ProfiledRotationPIDController/Output", output);
        if (atGoal)
            return 0;

        return output;
    }

    private ChassisSpeeds selfRelativeSpeedsFromFieldRelativePowers(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds fieldRelativeSpeeds = powersToSpeeds(xPower, yPower, thetaPower);
        return fieldRelativeSpeedsToSelfRelativeSpeeds(fieldRelativeSpeeds);
    }

    private ChassisSpeeds fieldRelativeSpeedsToSelfRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getDriveRelativeAngle());
    }

    private void updateHardware() {
        gyro.update();

        for (SwerveModule currentModule : swerveModules)
            currentModule.updatePeriodically();

        phoenix6SignalThread.updateLatestTimestamps();
    }

    private void updateNetworkTables() {
        final ChassisSpeeds selfRelativeVelocity = getSelfRelativeVelocity();
        Logger.recordOutput("Swerve/Velocity/OmegaRadiansPerSecond", selfRelativeVelocity.omegaRadiansPerSecond);
        Logger.recordOutput("Swerve/Velocity/XMetersPerSecond", selfRelativeVelocity.vxMetersPerSecond);
        Logger.recordOutput("Swerve/Velocity/YMetersPerSecond", selfRelativeVelocity.vyMetersPerSecond);
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++)
            states[i] = swerveModules[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
    @SuppressWarnings("unused")
    private SwerveModuleState[] getTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++)
            states[i] = swerveModules[i].getTargetState();

        return states;
    }
}