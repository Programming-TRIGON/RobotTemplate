package frc.trigon.robot.subsystems.poseestimator;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.robotposesources.PoseSourceConstants;
import frc.trigon.robot.robotposesources.RobotPoseSource;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Optional;

/**
 * A class that estimates the robot's pose using a {@link SwerveDrivePoseEstimator}, and robot pose sources.
 * This pose estimator will provide you the robot's pose relative to the current driver station.
 *
 * @author Shriqui - Captain, Omer - Programing Captain
 */
public class PoseEstimator implements AutoCloseable {
    private final Notifier periodicNotifier = new Notifier(this::periodic);
    private final Swerve swerve = Swerve.getInstance();
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private final Field2d field = new Field2d();
    private final RobotPoseSource[] robotPoseSources;
    private DriverStation.Alliance lastAlliance;

    /**
     * Constructs a new PoseEstimator.
     *
     * @param robotPoseSources the sources that should update the pose estimator apart from the odometry. This should be cameras etc.
     */
    public PoseEstimator(RobotPoseSource... robotPoseSources) {
        this.robotPoseSources = robotPoseSources;
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerve.getConstants().getKinematics(),
                swerve.getHeading(),
                swerve.getModulePositions(),
                PoseEstimatorConstants.DEFAULT_POSE,
                PoseEstimatorConstants.STATES_AMBIGUITY,
                PoseEstimatorConstants.VISION_CALCULATIONS_AMBIGUITY
        );

        putAprilTagsOnFieldWidget();
        DriverStation.getAlliance().ifPresent(alliance -> lastAlliance = alliance);
        periodicNotifier.startPeriodic(RobotConstants.PERIODIC_TIME_SECONDS);
    }

    @Override
    public void close() {
        field.close();
        periodicNotifier.close();
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to
     */
    public void resetPose(Pose2d currentPose) {
        final Pose2d currentBluePose = AllianceUtilities.toAlliancePose(currentPose);
        swerve.setHeading(currentBluePose.getRotation());

        resetPoseEstimator(currentBluePose);
    }

    /**
     * @return the estimated pose of the robot, relative to the current driver station
     */
    public Pose2d getCurrentPose() {
        return AllianceUtilities.toAlliancePose(swerveDrivePoseEstimator.getEstimatedPosition());
    }

    private void periodic() {
        updatePoseEstimator();
        if (didAllianceChange())
            updateFieldWidget();

        SmartDashboard.putData("field", field);
        Logger.recordOutput("robotPose", getCurrentPose());
    }

    private void updateFieldWidget() {
        putAprilTagsOnFieldWidget();
        DriverStation.getAlliance().ifPresent(alliance -> lastAlliance = alliance);
    }

    private boolean didAllianceChange() {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && lastAlliance != alliance.get();
    }

    private void resetPoseEstimator(Pose2d currentPose) {
        swerveDrivePoseEstimator.resetPosition(
                currentPose.getRotation(),
                swerve.getModulePositions(),
                currentPose
        );
    }

    private void updatePoseEstimator() {
        updatePoseEstimatorStates();
        attemptToUpdateWithRobotPoseSources();
        field.setRobotPose(getCurrentPose());
    }

    private void attemptToUpdateWithRobotPoseSources() {
        for (RobotPoseSource robotPoseSource : robotPoseSources) {
            if (robotPoseSource.hasNewResult())
                updateFromPoseSource(robotPoseSource);
        }
    }

    private void updateFromPoseSource(RobotPoseSource robotPoseSource) {
        final Pose2d robotPose = robotPoseSource.getRobotPose();

        field.getObject(robotPoseSource.getName()).setPose(AllianceUtilities.toAlliancePose(robotPose));
        swerveDrivePoseEstimator.addVisionMeasurement(
                robotPose,
                robotPoseSource.getLastResultTimestamp()
        );
    }

    private void updatePoseEstimatorStates() {
        swerveDrivePoseEstimator.update(swerve.getHeading(), swerve.getModulePositions());
    }

    private void putAprilTagsOnFieldWidget() {
        final HashMap<Integer, Pose3d> tagsIdToPose = PoseSourceConstants.TAG_ID_TO_POSE;

        for (Integer currentID : tagsIdToPose.keySet()) {
            final Pose2d tagPose = tagsIdToPose.get(currentID).toPose2d();
            field.getObject("Tag " + currentID).setPose(AllianceUtilities.toAlliancePose(tagPose));
        }
    }
}
