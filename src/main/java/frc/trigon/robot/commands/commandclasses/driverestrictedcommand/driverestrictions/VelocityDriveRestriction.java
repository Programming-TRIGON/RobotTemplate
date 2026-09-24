package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;

/**
 * A restriction that limits the robot's maximum linear and rotational velocity.
 * Used to slow down the robot for precise robot movement.
 */
public class VelocityDriveRestriction implements DriveRestriction {
    private final double maximumTranslationVelocityMetersPerSecond;
    private final double maximumRotationVelocityMetersPerSecond;

    /**
     * A restriction that limits the maximum linear and rotational velocity.
     *
     * @param maximumTranslationVelocityMetersPerSecond maximum linear velocity
     * @param maximumRotationVelocityRadiansPerSecond   maximum rotational velocity
     */
    public VelocityDriveRestriction(double maximumTranslationVelocityMetersPerSecond, Rotation2d maximumRotationVelocityRadiansPerSecond) {
        this.maximumTranslationVelocityMetersPerSecond = maximumTranslationVelocityMetersPerSecond;
        this.maximumRotationVelocityMetersPerSecond = maximumRotationVelocityRadiansPerSecond.getRadians();
    }

    @Override
    public Translation2d applyTranslationRestriction(Translation2d targetTranslation) {
        final double maximumTranslationVelocityPower = MathUtil.clamp(maximumTranslationVelocityMetersPerSecond / SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND, 0, 1);

        final Translation2d scaledRobotTargetTranslationPower = targetTranslation.times(maximumTranslationVelocityPower);
        final double scaledRobotTargetTranslationMagnitude = scaledRobotTargetTranslationPower.getNorm();

        if (scaledRobotTargetTranslationMagnitude > maximumTranslationVelocityPower)
            return scaledRobotTargetTranslationPower.times(maximumTranslationVelocityPower / scaledRobotTargetTranslationMagnitude);
        return scaledRobotTargetTranslationPower;
    }

    @Override
    public double applyRotationRestriction(double targetRotation) {
        final double maximumRotationVelocityPower = MathUtil.clamp(maximumRotationVelocityMetersPerSecond / SwerveConstants.MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND, 0, 1);
        return targetRotation * maximumRotationVelocityPower;
    }
}