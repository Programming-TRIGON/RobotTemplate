package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraInputsAutoLogged;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class AprilTagPhotonCameraIO extends AprilTagCameraIO {
    final PhotonCamera photonCamera;

    public AprilTagPhotonCameraIO(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);
    }

    @Override
    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = getLatestPipelineResult();

        inputs.hasResult = latestResult != null && latestResult.hasTargets();
        if (inputs.hasResult) {
            updateHasResultInputs(inputs, latestResult);
            return;
        }

        updateNoResultInputs(inputs);
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
    }

    private void updateHasResultInputs(AprilTagCameraInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        inputs.cameraSolvePNPPose = getSolvePNPPose(latestResult);
        inputs.latestResultTimestampSeconds = latestResult.getTimestampSeconds();
        inputs.visibleTagIDs = getVisibleTagIDs(latestResult);
        inputs.poseAmbiguity = latestResult.getMultiTagResult().isPresent() ? 0 : latestResult.getBestTarget().getPoseAmbiguity();
        inputs.distancesFromTags = getDistancesFromTags(latestResult);
    }

    private void updateNoResultInputs(AprilTagCameraInputsAutoLogged inputs) {
        inputs.cameraSolvePNPPose = new Pose3d();
        inputs.visibleTagIDs = new int[0];
    }

    /**
     * Estimates the camera's pose using Solve PNP using as many tags as possible.
     *
     * @param result the camera's pipeline result
     * @return the estimated pose
     */
    private Pose3d getSolvePNPPose(PhotonPipelineResult result) {
        if (result.getMultiTagResult().isPresent()) {
            final Transform3d cameraPoseTransform = result.getMultiTagResult().get().estimatedPose.best;
            return new Pose3d().plus(cameraPoseTransform).relativeTo(FieldConstants.APRIL_TAG_FIELD_LAYOUT.getOrigin());
        }

        final Pose3d tagPose = FieldConstants.TAG_ID_TO_POSE.get(result.getBestTarget().getFiducialId());
        final Transform3d targetToCamera = result.getBestTarget().getBestCameraToTarget().inverse();
        return tagPose.transformBy(targetToCamera);
    }

    private int[] getVisibleTagIDs(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final int[] visibleTagIDs = new int[targets.size()];

        for (int i = 0; i < targets.size(); i++)
            visibleTagIDs[i] = targets.get(i).getFiducialId();

        return visibleTagIDs;
    }

    private double[] getDistancesFromTags(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final double[] distances = new double[targets.size()];

        for (int i = 0; i < targets.size(); i++)
            distances[i] = getDistanceFromTarget(targets.get(i));

        return distances;
    }

    private double getDistanceFromTarget(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().getNorm();
    }
}
