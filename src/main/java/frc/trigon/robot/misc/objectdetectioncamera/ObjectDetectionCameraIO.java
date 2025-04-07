package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public class ObjectDetectionCameraIO {
    protected ObjectDetectionCameraIO() {
    }

    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class ObjectDetectionCameraInputs {
        /**
         * Whether there is at least one target or not for each game piece, by game piece index (type).
         */
        public boolean[] hasTarget = new boolean[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES];
        /**
         * Stores the Rotation3d of all visible objects.
         * The first index is the game piece ID (type).
         * The second index is the index of the game piece's Rotation3d, with the best object placed first (index 0).
         */
        public Rotation3d[][] visibleObjectRotations = new Rotation3d[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES][0];
        public double latestResultTimestamp = 0;
    }
}
