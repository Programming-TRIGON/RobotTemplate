package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.io.AprilTagLimelightIO;
import frc.trigon.robot.poseestimation.apriltagcamera.io.AprilTagPhotonCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.io.AprilTagSimulationCameraIO;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import frc.trigon.lib.hardware.RobotHardwareStats;

import java.util.function.BiFunction;

public class AprilTagCameraConstants {
    static final double
            MAXIMUM_DISTANCE_FROM_TAG_FOR_ACCURATE_RESULT_METERS = 5,
            MAXIMUM_DISTANCE_FROM_TAG_FOR_ACCURATE_SOLVE_PNP_RESULT_METERS = 2;
    static final Pose2d[] EMPTY_POSE_ARRAY = new Pose2d[0];
    static final double MAXIMUM_AMBIGUITY = 0.4;
    public static final PhotonPoseEstimator.ConstrainedSolvepnpParams CONSTRAINED_SOLVE_PNP_PARAMS = new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 0.1);

    public static final VisionSystemSim VISION_SIMULATION = RobotHardwareStats.isSimulation() ? new VisionSystemSim("VisionSimulation") : null;
    private static final int
            SIMULATION_CAMERA_RESOLUTION_WIDTH = 1600,
            SIMULATION_CAMERA_RESOLUTION_HEIGHT = 1200,
            SIMULATION_CAMERA_FPS = 60,
            SIMULATION_AVERAGE_CAMERA_LATENCY_MILLISECONDS = 35,
            SIMULATION_CAMERA_LATENCY_STANDARD_DEVIATIONS_MILLISECONDS = 5,
            SIMULATION_CAMERA_EXPOSURE_TIME_MILLISECONDS = 10;
    private static final Rotation2d SIMULATION_CAMERA_FOV = Rotation2d.fromDegrees(70);
    private static final double
            SIMULATION_CAMERA_AVERAGE_PIXEL_ERROR = 0.25,
            SIMULATION_CAMERA_PIXEL_STANDARD_DEVIATIONS = 0.08;
    public static final SimCameraProperties SIMULATION_CAMERA_PROPERTIES = new SimCameraProperties();

    static {
        if (RobotHardwareStats.isSimulation()) {
            configureSimulationCameraProperties();
            VISION_SIMULATION.addAprilTags(FieldConstants.APRIL_TAG_FIELD_LAYOUT);
        }
    }

    private static void configureSimulationCameraProperties() {
        SIMULATION_CAMERA_PROPERTIES.setCalibration(SIMULATION_CAMERA_RESOLUTION_WIDTH, SIMULATION_CAMERA_RESOLUTION_HEIGHT, SIMULATION_CAMERA_FOV);
        SIMULATION_CAMERA_PROPERTIES.setCalibError(SIMULATION_CAMERA_AVERAGE_PIXEL_ERROR, SIMULATION_CAMERA_PIXEL_STANDARD_DEVIATIONS);
        SIMULATION_CAMERA_PROPERTIES.setFPS(SIMULATION_CAMERA_FPS);
        SIMULATION_CAMERA_PROPERTIES.setAvgLatencyMs(SIMULATION_AVERAGE_CAMERA_LATENCY_MILLISECONDS);
        SIMULATION_CAMERA_PROPERTIES.setLatencyStdDevMs(SIMULATION_CAMERA_LATENCY_STANDARD_DEVIATIONS_MILLISECONDS);
        SIMULATION_CAMERA_PROPERTIES.setExposureTimeMs(SIMULATION_CAMERA_EXPOSURE_TIME_MILLISECONDS);
    }

    public enum AprilTagCameraType {
        PHOTON_CAMERA(AprilTagPhotonCameraIO::new),
        SIMULATION_CAMERA(AprilTagSimulationCameraIO::new),
        LIMELIGHT(AprilTagLimelightIO::new);

        final BiFunction<String, Transform3d, AprilTagCameraIO> createIOFunction;

        AprilTagCameraType(BiFunction<String, Transform3d, AprilTagCameraIO> createIOFunction) {
            this.createIOFunction = createIOFunction;
        }
    }
}