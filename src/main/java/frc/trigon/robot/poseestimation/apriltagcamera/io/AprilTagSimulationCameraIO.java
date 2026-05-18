package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.lib.utilities.DynamicCameraTransform;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraInputsAutoLogged;
import org.photonvision.simulation.PhotonCameraSim;

public class AprilTagSimulationCameraIO extends AprilTagPhotonCameraIO {
    private final PhotonCameraSim cameraSimulation;

    public AprilTagSimulationCameraIO(String cameraName, DynamicCameraTransform dynamicCameraTransform) {
        super(cameraName, dynamicCameraTransform);

        cameraSimulation = new PhotonCameraSim(photonCamera, AprilTagCameraConstants.SIMULATION_CAMERA_PROPERTIES);
        cameraSimulation.enableDrawWireframe(false);
        CommandScheduler.getInstance().schedule(GeneralCommands.getDelayedCommand(0.5, () -> AprilTagCameraConstants.VISION_SIMULATION.addCamera(cameraSimulation, dynamicCameraTransform.get3dRobotCenterToCamera(Timer.getFPGATimestamp()))));
    }

    @Override
    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
        AprilTagCameraConstants.VISION_SIMULATION.adjustCamera(cameraSimulation, dynamicCameraTransform.get3dRobotCenterToCamera(Timer.getFPGATimestamp()));
        super.updateInputs(inputs);
    }
}