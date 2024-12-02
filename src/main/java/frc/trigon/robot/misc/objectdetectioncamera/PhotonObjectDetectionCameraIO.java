package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation2d;
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

    private Rotation2d[] getVisibleObjectsYaw(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final Rotation2d[] visibleObjectsYaw = new Rotation2d[targets.size()];
        visibleObjectsYaw[0] = getBestTargetYaw(result);

        boolean hasSeenBestTarget = false;
        for (int i = 0; i < visibleObjectsYaw.length; i++) {
            final Rotation2d targetYaw = Rotation2d.fromDegrees(-targets.get(i).getYaw());
            if (targetYaw.equals(visibleObjectsYaw[0])) {
                hasSeenBestTarget = true;
                continue;
            }
            visibleObjectsYaw[hasSeenBestTarget ? i : i + 1] = targetYaw;
        }
        return visibleObjectsYaw;
    }

    private Rotation2d getBestTargetYaw(PhotonPipelineResult result) {
        double closestTargetDistance = Double.POSITIVE_INFINITY;
        Rotation2d bestTargetYaw = new Rotation2d();
        for (PhotonTrackedTarget target : result.getTargets()) {
            final double currentTargetDistance = Math.abs(target.getYaw()) + Math.abs(target.getPitch());
            if (closestTargetDistance > currentTargetDistance) {
                closestTargetDistance = currentTargetDistance;
                bestTargetYaw = Rotation2d.fromDegrees(-target.getYaw());
            }
        }
        return bestTargetYaw;
    }
}
