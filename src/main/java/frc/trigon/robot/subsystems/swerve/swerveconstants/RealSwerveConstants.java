package frc.trigon.robot.subsystems.swerve.swerveconstants;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class RealSwerveConstants extends SwerveConstants {
    private static final double
            GYRO_MOUNT_POSITION_YAW = 0,
            GYRO_MOUNT_POSITION_PITCH = 0,
            GYRO_MOUNT_POSITION_ROLL = 0;

    static final double
            MAX_SPEED_METERS_PER_SECOND = 4.04502,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;

    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(5, 0, 0),
            PROFILED_ROTATION_PID_CONSTANTS = new PIDConstants(5, 0, 0),
            AUTO_TRANSLATION_PID_CONSTANTS = new PIDConstants(6.5, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(3, 0, 0);
    private static final double
            MAX_ROTATION_VELOCITY = 720,
            MAX_ROTATION_ACCELERATION = 720;
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ROTATION_VELOCITY,
            MAX_ROTATION_ACCELERATION
    );
    private static final ProfiledPIDController PROFILED_ROTATION_PID_CONTROLLER = new ProfiledPIDController(
            PROFILED_ROTATION_PID_CONSTANTS.kP,
            PROFILED_ROTATION_PID_CONSTANTS.kI,
            PROFILED_ROTATION_PID_CONSTANTS.kD,
            ROTATION_CONSTRAINTS
    );
    private static final PIDController TRANSLATION_PID_CONTROLLER = new PIDController(
            TRANSLATION_PID_CONSTANTS.kP,
            TRANSLATION_PID_CONSTANTS.kI,
            TRANSLATION_PID_CONSTANTS.kD
    );

    @Override
    public double getMaxSpeedMetersPerSecond() {
        return MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    public double getMaxRotationalSpeedRadiansPerSecond() {
        return MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }

    @Override
    public ProfiledPIDController getProfiledRotationController() {
        return PROFILED_ROTATION_PID_CONTROLLER;
    }

    @Override
    public PIDController getTranslationsController() {
        return TRANSLATION_PID_CONTROLLER;
    }

    @Override
    public PIDConstants getAutoTranslationPIDConstants() {
        return AUTO_TRANSLATION_PID_CONSTANTS;
    }

    @Override
    public PIDConstants getAutoRotationPIDConstants() {
        return AUTO_ROTATION_PID_CONSTANTS;
    }

    @Override
    protected Pigeon2Configuration generateGyroConfiguration() {
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = GYRO_MOUNT_POSITION_YAW;
        config.MountPose.MountPosePitch = GYRO_MOUNT_POSITION_PITCH;
        config.MountPose.MountPoseRoll = GYRO_MOUNT_POSITION_ROLL;
        return config;
    }
}
