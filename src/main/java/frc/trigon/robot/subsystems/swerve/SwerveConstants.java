package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.simulationswerve.SimulationSwerveConstants;
import frc.trigon.robot.subsystems.swerve.trihardswerve.TrihardSwerveConstants;

public abstract class SwerveConstants {
    /**
     * This is the time the swerve should wait before braking after being disabled
     */
    static final double BRAKE_TIME_SECONDS = 4;
    static final int MAX_SAVED_PREVIOUS_LOOP_TIMESTAMPS = 10;
    static final double
            TRANSLATION_TOLERANCE = 0.03,
            ROTATION_TOLERANCE = 2,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.05;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0.1,
            ROTATION_NEUTRAL_DEADBAND = 0.1;

    public static SwerveConstants generateConstants() {
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIHARD)
            return new TrihardSwerveConstants();

        return new SimulationSwerveConstants();
    }

    /**
     * When the swerve moves, one of the motors might be slower than the others, thus the swerve will spin a bit when driving straight.
     * To counter this, we use pid to stay at the last angle the swerve's rotation moved at.
     *
     * @return the pid controller to stay at the last angle the swerve's rotation moved at
     */
    protected abstract PIDController getLookStraightController();

    protected abstract ProfiledPIDController getProfiledRotationController();

    /**
     * @return the swerve's robot side length in meters, (not including the bumpers)
     */
    protected abstract double getRobotSideLength();

    protected abstract SwerveModuleIO[] getModulesIO();

    public abstract SwerveDriveKinematics getKinematics();

    protected abstract HolonomicPathFollowerConfig getPathFollowerConfig();

    protected abstract double getMaxSpeedMetersPerSecond();

    protected abstract double getMaxRotationalSpeedRadiansPerSecond();

    protected abstract SlewRateLimiter getXSlewRateLimiter();

    protected abstract SlewRateLimiter getYSlewRateLimiter();
}
