package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraInputsAutoLogged;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class AprilTagPhotonCameraIO extends AprilTagCameraIO {
    private final Transform3d robotToCamera;
    final PhotonCamera photonCamera;

    public AprilTagPhotonCameraIO(String cameraName, Transform3d robotToCamera) {
        photonCamera = new PhotonCamera(cameraName);
        this.robotToCamera = robotToCamera;
    }

    @Override
    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = getLatestPipelineResult();

        inputs.hasResult = latestResult != null && latestResult.hasTargets();
        if (inputs.hasResult) {
            updateHasTargetInputs(inputs, latestResult);
            return;
        }

        updateNoTargetInputs(inputs);
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
    }

    private void updateHasTargetInputs(AprilTagCameraInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        final PhotonTrackedTarget[] visibleNotHatefulTags = getVisibleNotHatefulTags(latestResult);
        final PhotonTrackedTarget bestTag = visibleNotHatefulTags.length == 0 ? null : visibleNotHatefulTags[0];
        if (bestTag == null) {
            updateNoTargetInputs(inputs);
            inputs.hasResult = false;
            return;
        }

        updateSolvePNPPoses(inputs, latestResult, bestTag);
        if (inputs.bestCameraSolvePNPPose == null) {
            updateNoTargetInputs(inputs);
            inputs.hasResult = false;
            return;
        }

        inputs.latestResultTimestampSeconds = latestResult.getTimestampSeconds();
        inputs.visibleTagIDs = getVisibleTagIDs(visibleNotHatefulTags);
        inputs.poseAmbiguity = latestResult.getMultiTagResult().isPresent() ? 0 : bestTag.getPoseAmbiguity();
        inputs.distancesFromTags = getDistancesFromTags(visibleNotHatefulTags);
        inputs.hasConstrainedResult = false;
    }

    private void updateNoTargetInputs(AprilTagCameraInputsAutoLogged inputs) {
        inputs.bestCameraSolvePNPPose = new Pose3d();
        inputs.alternateCameraSolvePNPPose = inputs.bestCameraSolvePNPPose;
        inputs.constrainedSolvePNPPose = inputs.alternateCameraSolvePNPPose;
        inputs.visibleTagIDs = new int[0];
        inputs.hasConstrainedResult = false;
    }

    private PhotonTrackedTarget[] getVisibleNotHatefulTags(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final PhotonTrackedTarget[] visibleTagIDs = new PhotonTrackedTarget[targets.size()];
        int index = 0;

        for (PhotonTrackedTarget target : targets) {
            if (FieldConstants.TAG_ID_TO_POSE.containsKey(target.getFiducialId())) {
                visibleTagIDs[index] = target;
                index++;
            }
        }

        return Arrays.copyOf(visibleTagIDs, index);
    }

    private void updateSolvePNPPoses(AprilTagCameraInputsAutoLogged inputs, PhotonPipelineResult latestResult, PhotonTrackedTarget bestTag) {
        if (latestResult.getMultiTagResult().isPresent()) {
            final Transform3d tagToCamera = latestResult.getMultiTagResult().get().estimatedPose.best;
            inputs.bestCameraSolvePNPPose = FieldConstants.APRIL_TAG_FIELD_LAYOUT.getOrigin().transformBy(tagToCamera);
            inputs.alternateCameraSolvePNPPose = inputs.bestCameraSolvePNPPose;
            return;
        }

        final Pose3d tagPose = FieldConstants.TAG_ID_TO_POSE.get(bestTag.getFiducialId());
        if (tagPose == null) {
            inputs.bestCameraSolvePNPPose = null;
            return;
        }

        final Transform3d bestTagToCamera = bestTag.getBestCameraToTarget().inverse();
        final Transform3d alternateTagToCamera = bestTag.getAlternateCameraToTarget().inverse();

        inputs.bestCameraSolvePNPPose = tagPose.transformBy(bestTagToCamera);
        inputs.alternateCameraSolvePNPPose = tagPose.transformBy(alternateTagToCamera);

//        updateConstrainedSolvePNPPose(inputs, latestResult);
    }

    private void updateConstrainedSolvePNPPose(AprilTagCameraInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        final Pose3d constrainedSolvePNPPose = calculateConstrainedSolvePNPPose(latestResult, inputs.bestCameraSolvePNPPose);
        if (constrainedSolvePNPPose == null) {
            inputs.hasConstrainedResult = false;
            return;
        }

        inputs.constrainedSolvePNPPose = constrainedSolvePNPPose;
        inputs.hasConstrainedResult = true;
    }

    private Pose3d calculateConstrainedSolvePNPPose(PhotonPipelineResult result, Pose3d bestCameraSolvePNPPose) {
        final Optional<Matrix<N3, N3>> cameraMatrix = photonCamera.getCameraMatrix();
        final Optional<Matrix<N8, N1>> distCoeffs = photonCamera.getDistCoeffs();

        if (cameraMatrix.isEmpty() || distCoeffs.isEmpty())
            return null;

        Pose3d fieldToRobotSeed = bestCameraSolvePNPPose.transformBy(this.robotToCamera.inverse());
        final Rotation2d robotYawAtTimestamp = RobotContainer.ROBOT_POSE_ESTIMATOR.samplePoseAtTimestamp(result.getTimestampSeconds()).getRotation();

        if (!AprilTagCameraConstants.CONSTRAINED_SOLVE_PNP_PARAMS.headingFree()) {
            fieldToRobotSeed = new Pose3d(
                    fieldToRobotSeed.getTranslation(),
                    new Rotation3d(robotYawAtTimestamp)
            );
        }

        final Optional<PnpResult> pnpResult = VisionEstimation.estimateRobotPoseConstrainedSolvepnp(
                cameraMatrix.get(),
                distCoeffs.get(),
                result.getTargets(),
                robotToCamera,
                fieldToRobotSeed,
                FieldConstants.APRIL_TAG_FIELD_LAYOUT,
                TargetModel.kAprilTag36h11,
                AprilTagCameraConstants.CONSTRAINED_SOLVE_PNP_PARAMS.headingFree(),
                robotYawAtTimestamp,
                AprilTagCameraConstants.CONSTRAINED_SOLVE_PNP_PARAMS.headingScaleFactor()
        );

        return pnpResult.map(value -> Pose3d.kZero.plus(value.best).transformBy(robotToCamera)).orElse(null);
    }

    private int[] getVisibleTagIDs(PhotonTrackedTarget[] targets) {
        final int[] visibleTagIDs = new int[targets.length];

        for (int i = 0; i < visibleTagIDs.length; i++)
            visibleTagIDs[i] = targets[i].getFiducialId();

        return visibleTagIDs;
    }

    private double[] getDistancesFromTags(PhotonTrackedTarget[] targets) {
        final double[] distances = new double[targets.length];

        for (int i = 0; i < targets.length; i++)
            distances[i] = getDistanceFromTarget(targets[i]);

        return distances;
    }

    private double getDistanceFromTarget(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().getNorm();
    }
}