package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraInputsAutoLogged;
import frc.trigon.lib.utilities.LimelightHelpers;

// TODO: Fully implement this class, Limelight currently not supported.
public class AprilTagLimelightIO extends AprilTagCameraIO {
    private final String hostname;

    public AprilTagLimelightIO(String hostname, Transform3d robotToCamera) {
        this.hostname = hostname;
    }

    @Override
    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
        final LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(hostname);

        inputs.hasResult = results.targets_Fiducials.length > 0;

        if (inputs.hasResult) {
            updateHasTargetInputs(inputs, results);
            return;
        }
        updateNoTargetInputs(inputs);
    }

    private void updateHasTargetInputs(AprilTagCameraInputsAutoLogged inputs, LimelightHelpers.LimelightResults results) {
        inputs.bestCameraSolvePNPPose = results.getBotPose3d_wpiBlue();
        inputs.latestResultTimestampSeconds = results.timestamp_RIOFPGA_capture;
        inputs.visibleTagIDs = getVisibleTagIDs(results);
    }

    private void updateNoTargetInputs(AprilTagCameraInputsAutoLogged inputs) {
        inputs.bestCameraSolvePNPPose = new Pose3d();
        inputs.visibleTagIDs = new int[0];
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

    private LimelightHelpers.LimelightTarget_Fiducial getBestTarget(LimelightHelpers.LimelightResults results) {
        return results.targets_Fiducials[0];
    }
}