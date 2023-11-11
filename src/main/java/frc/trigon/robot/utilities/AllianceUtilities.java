package frc.trigon.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.robot.constants.FieldConstants;

import java.util.Optional;

public class AllianceUtilities {
    /**
     * @return whether the robot is on the blue alliance
     */
    public static boolean isBlueAlliance() {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Blue).isPresent();
    }

    /**
     * Converts a pose to the pose relative to the current driver station alliance.
     *
     * @param pose the current blue alliance pose
     * @return the converted pose
     */
    public static Pose2d toAlliancePose(Pose2d pose) {
        if (isBlueAlliance())
            return pose;

        return new Pose2d(
                FieldConstants.FIELD_LENGTH_METERS - pose.getX(),
                FieldConstants.FIELD_WIDTH_METERS - pose.getY(),
                pose.getRotation().minus(Rotation2d.fromRotations(0.5))
        );
    }
}
