package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import frc.trigon.lib.utilities.flippable.FlippablePose3d;

import java.util.ArrayList;
import java.util.List;

public class SimulationScoringHandler {
    public static void checkGamePieceScored(SimulatedGamePiece gamePiece) {
        final ArrayList<FlippablePose3d> scoreLocations = new ArrayList<>(List.of(SimulatedGamePieceConstants.SCORING_LOCATION));
        for (FlippablePose3d scoreLocation : scoreLocations) {
            final Pose3d flippedPose = scoreLocation.get();
            if (isGamePieceScored(gamePiece, flippedPose, SimulatedGamePieceConstants.SCORING_TOLERANCE_METERS)) {
                gamePiece.isScored = true;
                gamePiece.updatePose(flippedPose);
                scoreLocations.remove(scoreLocation);
                return;
            }
        }
    }

    private static boolean isGamePieceScored(SimulatedGamePiece gamePiece, Pose3d scoreLocation, double scoringToleranceMeters) {
        final double distanceFromScoreZoneMeters = gamePiece.getDistanceFromPoseMeters(scoreLocation);
        return distanceFromScoreZoneMeters < scoringToleranceMeters;
    }
}
