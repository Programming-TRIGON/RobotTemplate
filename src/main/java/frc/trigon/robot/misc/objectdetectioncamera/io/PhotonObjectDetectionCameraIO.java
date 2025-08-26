package frc.trigon.robot.misc.objectdetectioncamera.io;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraIO;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraInputsAutoLogged;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PhotonObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private final PhotonCamera photonCamera;

    public PhotonObjectDetectionCameraIO(String hostname) {
        PhotonCamera.setVersionCheckEnabled(false);
        photonCamera = new PhotonCamera(hostname);
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        if (!photonCamera.isConnected()) {
            updateNoNewResultInputs(inputs);
            return;
        }

        final PhotonPipelineResult result = getLatestPipelineResult();
        if (result == null || !result.hasTargets()) {
            updateNoNewResultInputs(inputs);
            return;
        }

        updateHasNewResultInputs(inputs, result);
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
    }

    private void updateNoNewResultInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        inputs.hasTarget = new boolean[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES];
        inputs.visibleObjectRotations = new Rotation3d[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES][0];
    }

    private void updateHasNewResultInputs(ObjectDetectionCameraInputsAutoLogged inputs, PhotonPipelineResult result) {
        final List<Rotation3d>[] visibleObjectsRotations = new List[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES];
        for (int i = 0; i < ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES; i++)
            visibleObjectsRotations[i] = new ArrayList<>();
        Arrays.fill(inputs.hasTarget, false);
        inputs.latestResultTimestamp = result.getTimestampSeconds();

        for (PhotonTrackedTarget currentTarget : result.getTargets()) {
            if (currentTarget.getDetectedObjectClassID() == -1)
                continue;

            inputs.hasTarget[currentTarget.getDetectedObjectClassID()] = true;
            visibleObjectsRotations[currentTarget.getDetectedObjectClassID()].add(extractRotation3d(currentTarget));
        }

        for (int i = 0; i < ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES; i++)
            inputs.visibleObjectRotations[i] = toArray(visibleObjectsRotations[i]);
    }

    private Rotation3d[] toArray(List<Rotation3d> list) {
        final Rotation3d[] array = new Rotation3d[list.size()];

        for (int i = 0; i < array.length; i++)
            array[i] = list.get(i);

        return array;
    }

    private Rotation3d extractRotation3d(PhotonTrackedTarget target) {
        return new Rotation3d(
                0,
                Units.degreesToRadians(-target.getPitch()),
                Units.degreesToRadians(-target.getYaw())
        );
    }
}
