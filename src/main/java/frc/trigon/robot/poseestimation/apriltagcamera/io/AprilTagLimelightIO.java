package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraInputsAutoLogged;
import org.trigon.utilities.LimelightHelpers;

public class AprilTagLimelightIO extends AprilTagCameraIO {
    private final String hostname;

    public AprilTagLimelightIO(String hostname) {
        this.hostname = hostname;
    }

    @Override
    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
        final LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(hostname);

        inputs.hasResult = results.targets_Fiducials.length > 0;

        if (inputs.hasResult) {
            updateHasResultInputs(inputs, results);
            return;
        }
        updateNoResultInputs(inputs);
    }

    private void updateHasResultInputs(AprilTagCameraInputsAutoLogged inputs, LimelightHelpers.LimelightResults results) {
        final Rotation3d bestTagRelativeRotation = getBestTargetRelativeRotation(results);

        inputs.cameraSolvePNPPose = results.getBotPose3d_wpiBlue();
        inputs.latestResultTimestampSeconds = results.timestamp_RIOFPGA_capture;
        inputs.visibleTagIDs = getVisibleTagIDs(results);
        inputs.bestTargetRelativeYawRadians = bestTagRelativeRotation.getZ();
        inputs.bestTargetRelativePitchRadians = bestTagRelativeRotation.getY();
        inputs.distanceFromBestTag = getDistanceFromBestTag(results);
    }

    private void updateNoResultInputs(AprilTagCameraInputsAutoLogged inputs) {
        inputs.cameraSolvePNPPose = new Pose3d();
        inputs.visibleTagIDs = new int[0];
        inputs.distanceFromBestTag = Double.POSITIVE_INFINITY;
    }

    private int[] getVisibleTagIDs(LimelightHelpers.LimelightResults results) {
        final LimelightHelpers.LimelightTarget_Fiducial[] visibleTags = results.targets_Fiducials;
        final int[] visibleTagIDs = new int[visibleTags.length];
        visibleTagIDs[0] = (int) getBestTarget(results).fiducialID;
        int idAddition = 1;

        for (int i = 0; i < visibleTagIDs.length; i++) {
            final int currentID = (int) visibleTags[i].fiducialID;

            if (currentID == visibleTagIDs[0]) {
                idAddition = 0;
                continue;
            }

            visibleTagIDs[i + idAddition] = currentID;
        }
        return visibleTagIDs;
    }

    /**
     * Estimates the camera's rotation relative to the apriltag.
     *
     * @param results the camera's pipeline result
     * @return the estimated rotation
     */
    private Rotation3d getBestTargetRelativeRotation(LimelightHelpers.LimelightResults results) {
        final LimelightHelpers.LimelightTarget_Fiducial bestTag = getBestTarget(results);
        return new Rotation3d(0, -Units.degreesToRadians(bestTag.tx), -Units.degreesToRadians(bestTag.ty));
    }

    private double getDistanceFromBestTag(LimelightHelpers.LimelightResults results) {
        return getDistanceFromTag((int) getBestTarget(results).fiducialID, results.getBotPose3d_wpiBlue());
    }

    private double getDistanceFromTag(int fiducialID, Pose3d estimatedRobotPose) {
        return FieldConstants.TAG_ID_TO_POSE.get(fiducialID).getTranslation().getDistance(estimatedRobotPose.getTranslation());
    }

    private LimelightHelpers.LimelightTarget_Fiducial getBestTarget(LimelightHelpers.LimelightResults results) {
        return results.targets_Fiducials[0];
    }
}