package frc.trigon.robot.subsystems.swerve;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class Swerve extends SubsystemBase {
    private static final Swerve INSTANCE = new Swerve();
    private static final PoseEstimator POSE_ESTIMATOR = RobotContainer.POSE_ESTIMATOR;
    private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();
    private final SwerveIO swerveIO;
    private final SwerveModuleIO[] modulesIO;
    private final SwerveConstants constants;
    private final Notifier updateLastRotationMovementAngleNotifier = new Notifier(() -> lastRotationMovementAngle = POSE_ESTIMATOR.getCurrentPose().getRotation());
    private final Notifier configurePathPlannerNotifier = new Notifier(this::configurePathPlanner);
    private final List<Double> previousLoopTimestamps = new ArrayList<>();
    private Pose2d targetProfiledPose = null;
    private Rotation2d lastRotationMovementAngle = new Rotation2d();

    public static Swerve getInstance() {
        return INSTANCE;
    }

    private Swerve() {
        swerveIO = SwerveIO.generateIO();
        constants = SwerveConstants.generateConstants();
        modulesIO = getModulesIO();
        configurePathPlannerNotifier.startSingle(2);
    }

    @Override
    public void periodic() {
        swerveIO.updateInputs(swerveInputs);
        Logger.processInputs("Swerve", swerveInputs);

        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.periodic();

        updateNetworkTables();
    }

    public SwerveConstants getConstants() {
        return constants;
    }

    public double getGyroZAcceleration() {
        return swerveInputs.accelerationZ;
    }

    public double getGyroYAcceleration() {
        return swerveInputs.accelerationY;
    }

    public double getGyroXAcceleration() {
        return swerveInputs.accelerationX;
    }

    public Rotation2d getHeading() {
        final double heading = swerveInputs.gyroYawDegrees;
        final double inputtedHeading = MathUtil.inputModulus(heading, -180, 180);

        return Rotation2d.fromDegrees(inputtedHeading);
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return constants.getKinematics().toChassisSpeeds(getModuleStates());
    }

    public void setHeading(Rotation2d heading) {
        swerveIO.setHeading(heading);
        lastRotationMovementAngle = AllianceUtilities.isBlueAlliance() ? heading : heading.minus(Rotation2d.fromRotations(0.5));
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(swerveInputs.gyroPitchDegrees);
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(swerveInputs.gyroRollDegrees);
    }

    public Rotation2d getPitchVelocity() {
        return Rotation2d.fromDegrees(swerveInputs.gyroPitchVelocity);
    }

    public Rotation2d getRollVelocity() {
        return Rotation2d.fromDegrees(swerveInputs.gyroRollVelocity);
    }

    public SwerveModulePosition[] getModulePositions() {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            swerveModulePositions[i] = modulesIO[i].getCurrentPosition();

        return swerveModulePositions;
    }

    public boolean atPose(Pose2d pose2d) {
        return atXAxisPosition(pose2d.getX()) && atYAxisPosition(pose2d.getY()) && atAngle(pose2d.getRotation());
    }

    public boolean atXAxisPosition(double xAxisPosition) {
        final double currentXAxisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(getSelfRelativeVelocity(), POSE_ESTIMATOR.getCurrentPose().getRotation()).vxMetersPerSecond;
        return atTranslationPosition(POSE_ESTIMATOR.getCurrentPose().getX(), xAxisPosition, currentXAxisVelocity);
    }

    public boolean atYAxisPosition(double yAxisPosition) {
        final double currentYAxisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(getSelfRelativeVelocity(), POSE_ESTIMATOR.getCurrentPose().getRotation()).vyMetersPerSecond;
        return atTranslationPosition(POSE_ESTIMATOR.getCurrentPose().getY(), yAxisPosition, currentYAxisVelocity);
    }

    public boolean atAngle(Rotation2d angle) {
        return Math.abs(angle.getDegrees() - POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees()) < SwerveConstants.ROTATION_TOLERANCE &&
                Math.abs(getSelfRelativeVelocity().omegaRadiansPerSecond) < SwerveConstants.ROTATION_VELOCITY_TOLERANCE;
    }

    void initializeDrive(boolean closedLoop) {
        setBrake(true);
        setClosedLoop(closedLoop);
        setLastRotationMovementAngle(POSE_ESTIMATOR.getCurrentPose().getRotation());
    }

    /**
     * Locks the swerve, so it'll be hard to move it. This will make the modules look in the middle of a robot in a "x" shape.
     */
    void lockSwerve() {
        setBrake(true);
        final SwerveModuleState
                right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

        modulesIO[0].setTargetState(left);
        modulesIO[1].setTargetState(right);
        modulesIO[2].setTargetState(right);
        modulesIO[3].setTargetState(left);
    }

    /**
     * When the swerve moves, one of the motors might be slower than the others, thus the swerve will spin a bit when driving straight.
     * To counter this, we use pid to stay at the last angle the swerve's rotation moved at.
     *
     * @param lastRotationMovementAngle the angle the swerve will try to stay at when it gets no rotation power
     */
    void setLastRotationMovementAngle(Rotation2d lastRotationMovementAngle) {
        this.lastRotationMovementAngle = lastRotationMovementAngle;
    }

    void resetRotationController() {
        constants.getProfiledRotationController().reset(POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees());
    }

    void setClosedLoop(boolean closedLoop) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setDriveMotorClosedLoop(closedLoop);
    }

    void stop() {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.stop();
    }

    /**
     * Sets whether the swerve motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     */
    void setBrake(boolean brake) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setBrake(brake);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the field's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle
     * @param rateLimit   whether the swerve should be rate limited
     */
    void fieldRelativeDrive(double xPower, double yPower, Rotation2d targetAngle, boolean rateLimit) {
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, 0, rateLimit);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);

        selfRelativeDrive(speeds);
        updateTargetProfiledPoseFromRotationController();
    }

    /**
     * Drives the swerve with the given powers, relative to the field's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     * @param rateLimit  whether the swerve should be rate limited
     */
    void fieldRelativeDrive(double xPower, double yPower, double thetaPower, boolean rateLimit) {
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, thetaPower, rateLimit);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the robot's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     * @param rateLimit  whether the swerve should be rate limited
     */
    void selfRelativeDrive(double xPower, double yPower, double thetaPower, boolean rateLimit) {
        final ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, thetaPower);
        if (rateLimit)
            rateLimit(speeds);

        selfRelativeDrive(speeds);
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        updatePreviousLoopTimestamps();

        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        chassisSpeeds = discretize(chassisSpeeds);
        final SwerveModuleState[] swerveModuleStates = constants.getKinematics().toSwerveModuleStates(chassisSpeeds);
        setTargetModuleStates(swerveModuleStates);
    }

    private void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < modulesIO.length; i++)
            modulesIO[i].setTargetState(swerveModuleStates[i]);
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds the chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    private ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        return ChassisSpeeds.discretize(chassisSpeeds, getAverageLoopTime());
    }

    private double getAverageLoopTime() {
        double differenceSum = 0;
        double lastTimestamp = -1;

        for (double currentTimestamp : previousLoopTimestamps) {
            if (lastTimestamp == -1) {
                lastTimestamp = currentTimestamp;
                continue;
            }
            differenceSum += currentTimestamp - lastTimestamp;
            lastTimestamp = currentTimestamp;
        }

        return differenceSum / (previousLoopTimestamps.size() - 1);
    }

    private void updatePreviousLoopTimestamps() {
        if (previousLoopTimestamps.size() < SwerveConstants.MAX_SAVED_PREVIOUS_LOOP_TIMESTAMPS) {
            previousLoopTimestamps.add(Timer.getFPGATimestamp());
            return;
        }

        previousLoopTimestamps.remove(0);
        previousLoopTimestamps.add(Timer.getFPGATimestamp());
    }

    private void configurePathPlanner() {
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> this.targetProfiledPose = pose);
        AutoBuilder.configureHolonomic(
                POSE_ESTIMATOR::getCurrentPose,
                POSE_ESTIMATOR::resetPose,
                this::getSelfRelativeVelocity,
                this::selfRelativeDrive,
                constants.getPathFollowerConfig(),
                this
        );

        configurePathPlannerNotifier.close();
    }

    private boolean atTranslationPosition(double currentTranslationPosition, double targetTranslationPosition, double currentTranslationVelocity) {
        return Math.abs(currentTranslationPosition - targetTranslationPosition) < SwerveConstants.TRANSLATION_TOLERANCE &&
                Math.abs(currentTranslationVelocity) < SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    private double calculateProfiledAngleSpeedToTargetAngle(Rotation2d targetAngle) {
        constants.getProfiledRotationController().setGoal(targetAngle.getDegrees());
        final Rotation2d currentAngle = POSE_ESTIMATOR.getCurrentPose().getRotation();
        return Units.degreesToRadians(constants.getProfiledRotationController().calculate(currentAngle.getDegrees()));
    }

    private void updateTargetProfiledPoseFromRotationController() {
        targetProfiledPose = new Pose2d(
                POSE_ESTIMATOR.getCurrentPose().getTranslation(),
                Rotation2d.fromDegrees(constants.getProfiledRotationController().getSetpoint().position)
        );
    }

    private ChassisSpeeds selfRelativeSpeedsFromFieldRelativePowers(double xPower, double yPower, double thetaPower, boolean rateLimit) {
        final ChassisSpeeds fieldRelativeSpeeds = powersToSpeeds(xPower, yPower, thetaPower);
        final ChassisSpeeds selfRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, POSE_ESTIMATOR.getCurrentPose().getRotation());
        if (rateLimit)
            rateLimit(selfRelativeSpeeds);
        return selfRelativeSpeeds;
    }

    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * constants.getMaxSpeedMetersPerSecond(),
                yPower * constants.getMaxSpeedMetersPerSecond(),
                getDriveSpeedFromPower(thetaPower)
        );
    }

    private double getDriveSpeedFromPower(double rotPower) {
        if (rotPower == 0)
            return getLockAtLastRotationMovementAngleSpeed();

        updateLastRotationMovement();
        return Math.pow(rotPower, 2) * Math.signum(rotPower) * constants.getMaxRotationalSpeedRadiansPerSecond();
    }

    private void updateLastRotationMovement() {
        updateLastRotationMovementAngleNotifier.stop();
        updateLastRotationMovementAngleNotifier.startSingle(0.3);
        lastRotationMovementAngle = null;
    }

    /**
     * When the swerve moves, one of the motors might be slower than the others, thus the swerve will spin a bit when driving straight.
     * To counter this, we use pid to stay at the last angle the swerve's rotation moved at.
     *
     * @return the pid output to stay at the last angle the swerve's rotation moved at, in radians per second
     */
    private double getLockAtLastRotationMovementAngleSpeed() {
        if (lastRotationMovementAngle == null)
            return 0;
        final Rotation2d currentAngle = POSE_ESTIMATOR.getCurrentPose().getRotation();
        final double outputDegrees = constants.getRotationController().calculate(currentAngle.getDegrees(), lastRotationMovementAngle.getDegrees());
        return Units.degreesToRadians(outputDegrees);
    }

    private void rateLimit(ChassisSpeeds toLimit) {
        final double x = toLimit.vxMetersPerSecond;
        final double y = toLimit.vyMetersPerSecond;
        toLimit.vxMetersPerSecond = x == 0 ? constants.getXSlewRateLimiter().calculate(x) : x;
        toLimit.vyMetersPerSecond = y == 0 ? constants.getYSlewRateLimiter().calculate(y) : y;
    }

    private void updateNetworkTables() {
        Logger.recordOutput("Swerve/Velocity/rot", getSelfRelativeVelocity().omegaRadiansPerSecond);
        Logger.recordOutput("Swerve/Velocity/x", getSelfRelativeVelocity().vxMetersPerSecond);
        Logger.recordOutput("Swerve/Velocity/y", getSelfRelativeVelocity().vyMetersPerSecond);

        if (targetProfiledPose == null) {
            Logger.recordOutput("targetPose", POSE_ESTIMATOR.getCurrentPose());
        } else {
            Logger.recordOutput("targetPose", targetProfiledPose);
            targetProfiledPose = null;
        }
    }

    @AutoLogOutput(key = "Swerve/currentStates")
    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Swerve/targetStates")
    private SwerveModuleState[] getTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getTargetState();

        return states;
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return
                Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                        Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                        Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    private SwerveModuleIO[] getModulesIO() {
        if (RobotConstants.IS_REPLAY) {
            return new SwerveModuleIO[]{
                    new SwerveModuleIO("FrontLeft"),
                    new SwerveModuleIO("FrontRight"),
                    new SwerveModuleIO("RearLeft"),
                    new SwerveModuleIO("RearRight")
            };
        }

        return constants.getModulesIO();
    }
}