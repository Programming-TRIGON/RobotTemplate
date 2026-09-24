package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;

/**
 * A drive restriction that limits the robot's linear and rotational acceleration.
 */
public class AccelerationDriveRestriction implements DriveRestriction {
    private final double maximumTranslationAccelerationPower;
    private final SlewRateLimiter rotationAccelerationRadiansPerSecondSquaredLimiter;
    private Translation2d currentTranslationAcceleration = Translation2d.kZero;

    /**
     * A drive restriction that limits the maximum linear and rotational acceleration.
     *
     * @param maximumTranslationAccelerationMetersPerSecondSquared maximum linear acceleration
     * @param maximumRotationAccelerationRadiansPerSecondSquared   maximum rotational acceleration
     */
    public AccelerationDriveRestriction(double maximumTranslationAccelerationMetersPerSecondSquared, double maximumRotationAccelerationRadiansPerSecondSquared) {
        this.maximumTranslationAccelerationPower = maximumTranslationAccelerationMetersPerSecondSquared / SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND;
        this.rotationAccelerationRadiansPerSecondSquaredLimiter = new SlewRateLimiter(maximumRotationAccelerationRadiansPerSecondSquared / SwerveConstants.MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND);
    }


    @Override
    public void init() {
        final Translation2d currentVelocity = RobotContainer.SWERVE.getFieldRelativeVelocity().div(SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND);
        currentTranslationAcceleration = Flippable.isRedAlliance() ? currentVelocity.unaryMinus() : currentVelocity;
        rotationAccelerationRadiansPerSecondSquaredLimiter.reset(RobotContainer.SWERVE.getRotationalVelocityRadiansPerSecond() / SwerveConstants.MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND);
    }

    @Override
    public Translation2d applyTranslationRestriction(Translation2d targetTranslation) {
        currentTranslationAcceleration = MathUtil.slewRateLimit(
                currentTranslationAcceleration,
                targetTranslation,
                RobotHardwareStats.getPeriodicTimeSeconds(),
                maximumTranslationAccelerationPower
        );
        return currentTranslationAcceleration;
    }

    @Override
    public double applyRotationRestriction(double targetRotation) {
        return rotationAccelerationRadiansPerSecondSquaredLimiter.calculate(targetRotation);
    }
}