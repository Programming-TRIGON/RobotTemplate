package frc.trigon.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        return switchAlliance(pose);
    }

    private static Pose2d switchAlliance(Pose2d pose) {
        return new Pose2d(
                FieldConstants.FIELD_LENGTH_METERS - pose.getX(),
                FieldConstants.FIELD_WIDTH_METERS - pose.getY(),
                pose.getRotation().minus(Rotation2d.fromRotations(0.5))
        );
    }

    public static class AlliancePose2d {
        private final Pose2d blueAlliancePose;
        private final Pose2d redAlliancePose;
        private Pose2d currentAlliancePose = null;

        private AlliancePose2d(Pose2d blueAlliancePose) {
            this.blueAlliancePose = blueAlliancePose;
            redAlliancePose = switchAlliance(blueAlliancePose);
        }

        public AlliancePose2d() {
            this(new Pose2d());
        }

        public static AlliancePose2d fromBlueAlliancePose(Pose2d blueAlliancePose) {
            return new AlliancePose2d(blueAlliancePose);
        }

        public static AlliancePose2d fromBlueAlliancePose(Translation2d translation, Rotation2d rotation) {
            return fromBlueAlliancePose(new Pose2d(translation, rotation));
        }

        public static AlliancePose2d fromBlueAlliancePose(double x, double y, Rotation2d rotation) {
            return fromBlueAlliancePose(new Pose2d(x, y, rotation));
        }

        public static AlliancePose2d fromRedAlliancePose(Pose2d redAlliancePose) {
            return new AlliancePose2d(switchAlliance(redAlliancePose));
        }

        public static AlliancePose2d fromRedAlliancePose(Translation2d translation, Rotation2d rotation) {
            return fromRedAlliancePose(new Pose2d(translation, rotation));
        }

        public static AlliancePose2d fromRedAlliancePose(double x, double y, Rotation2d rotation) {
            return fromRedAlliancePose(new Pose2d(x, y, rotation));
        }

        public static AlliancePose2d fromCurrentAlliancePose(Pose2d currentAlliancePose) {
            return new AlliancePose2d(toAlliancePose(currentAlliancePose));
        }

        public static AlliancePose2d fromCurrentAlliancePose(Translation2d translation, Rotation2d rotation) {
            return fromCurrentAlliancePose(new Pose2d(translation, rotation));
        }

        public static AlliancePose2d fromCurrentAlliancePose(double x, double y, Rotation2d rotation) {
            return fromCurrentAlliancePose(new Pose2d(x, y, rotation));
        }

        public Pose2d toBlueAlliancePose() {
            return blueAlliancePose;
        }

        public Pose2d toRedAlliancePose() {
            return redAlliancePose;
        }

        public Pose2d toCurrentAlliancePose() {
            if (currentAlliancePose == null)
                currentAlliancePose = toAlliancePose(blueAlliancePose);
            return currentAlliancePose;
        }
    }
}
