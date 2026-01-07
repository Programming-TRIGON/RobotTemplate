package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import org.photonvision.simulation.PhotonCameraSim;

public class AprilTagSimulationCameraIO extends AprilTagPhotonCameraIO {

    public AprilTagSimulationCameraIO(String cameraName, Transform3d robotToCamera) {
        super(cameraName, robotToCamera);

        final PhotonCameraSim cameraSimulation = new PhotonCameraSim(photonCamera, AprilTagCameraConstants.SIMULATION_CAMERA_PROPERTIES);
        cameraSimulation.enableDrawWireframe(true);
        AprilTagCameraConstants.VISION_SIMULATION.addCamera(cameraSimulation, robotToCamera);
    }
}