package frc.trigon.robot.misc.objectdetectioncamera;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class PhotonObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private final PhotonCamera photonCamera;

    protected PhotonObjectDetectionCameraIO(String hostname) {
        PhotonCamera.setVersionCheckEnabled(false);
        photonCamera = new PhotonCamera(hostname);
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        if (!photonCamera.isConnected())
            return;
        final PhotonPipelineResult result = getLatestPipelineResult();

        inputs.hasTargets = result != null && result.hasTargets();
        if (inputs.hasTargets) {
            inputs.visibleObjectsYaw = getVisibleObjectsYaw(result);
        }
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
    }

    private double[] getVisibleObjectsYaw(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final double[] visibleObjectsYaw = new double[targets.size()];
        visibleObjectsYaw[0] = getBestTargetYaw(result);

        boolean hasSeenBestTarget = false;
        for (int i = 0; i < visibleObjectsYaw.length; i++) {
            final double targetYaw = -targets.get(i).getYaw();
            if (targetYaw == visibleObjectsYaw[0]) {
                hasSeenBestTarget = true;
                continue;
            }
            visibleObjectsYaw[hasSeenBestTarget ? i : i + 1] = targetYaw;
        }
        return visibleObjectsYaw;
    }

    private double getBestTargetYaw(PhotonPipelineResult result) {
        double closestTargetDistance = Double.POSITIVE_INFINITY;
        double bestTargetYaw = 0;
        for (PhotonTrackedTarget target : result.getTargets()) {
            final double currentTargetDistance = Math.abs(target.getYaw()) + Math.abs(target.getPitch());
            if (closestTargetDistance > currentTargetDistance) {
                closestTargetDistance = currentTargetDistance;
                bestTargetYaw = -target.getYaw();
            }
        }
        return bestTargetYaw;
    }
}
