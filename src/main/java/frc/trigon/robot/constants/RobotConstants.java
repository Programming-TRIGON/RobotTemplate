package frc.trigon.robot.constants;

import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.FilesHandler;
import frc.trigon.robot.Robot;

public class RobotConstants {
    public static final boolean USE_CANIVORE = RobotHardwareStats.isSimulation() ? true : false;
    public static final String CANIVORE_NAME = USE_CANIVORE ? "SwerveCANivore" : "";
    public static final String LOGGING_PATH = Robot.IS_REAL ? "/media/sda1/akitlogs/" : FilesHandler.DEPLOY_PATH + "logs/";
    private static final RobotHardwareStats.ReplayType REPLAY_TYPE = RobotHardwareStats.ReplayType.NONE;
    private static final double PERIODIC_TIME_SECONDS = 0.02;

    public static void init() {
        RobotHardwareStats.setCurrentRobotStats(Robot.IS_REAL, REPLAY_TYPE);
        RobotHardwareStats.setPeriodicTimeSeconds(PERIODIC_TIME_SECONDS);
    }
}
