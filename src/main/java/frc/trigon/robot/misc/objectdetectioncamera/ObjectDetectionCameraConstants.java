package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;

public class ObjectDetectionCameraConstants {
    public static final int NUMBER_OF_GAME_PIECE_TYPES = SimulatedGamePieceConstants.GamePieceType.values().length;
    static final double TRACKED_OBJECT_TOLERANCE_METERS = 0.12;
    public static final Rotation2d LOLLIPOP_TOLERANCE = Rotation2d.fromDegrees(3.5);
}
