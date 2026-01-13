package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.trigon.lib.utilities.flippable.FlippablePose3d;
import frc.trigon.lib.utilities.flippable.FlippableTranslation2d;

import java.util.ArrayList;
import java.util.List;

public class SimulatedGamePieceConstants {
    public static final double G_FORCE = 9.806;

    public static final double
            FEEDER_INTAKE_TOLERANCE_METERS = 0.1,
            INTAKE_TOLERANCE_METERS = 0.1,
            SCORING_TOLERANCE_METERS = 0.1;

    /**
     * Stores all the game pieces.
     * Starts out with the game pieces the start on the field.
     */
    public static final ArrayList<SimulatedGamePiece>
            STARTING_GAME_PIECES = new ArrayList<>(List.of(
    ));

    public static final FlippablePose3d SCORING_LOCATION = new FlippablePose3d(0, 0, 0, new Rotation3d(), true);
    public static final FlippableTranslation2d FEEDER_POSITION = new FlippableTranslation2d(0, 0, true);

    public enum GamePieceType {
        GAME_PIECE_TYPE(0, 0);

        public final double originPointHeightOffGroundMeters;
        public final int id;

        GamePieceType(double originPointHeightOffGroundMeters, int id) {
            this.originPointHeightOffGroundMeters = originPointHeightOffGroundMeters;
            this.id = id;
        }

        public static String getNameFromID(int id) {
            for (int i = 0; i < values().length; i++)
                if (values()[i].id == id)
                    return values()[i].toString();
            return "";
        }
    }
}
