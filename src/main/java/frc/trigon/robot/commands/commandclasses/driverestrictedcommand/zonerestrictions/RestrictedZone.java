package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.zonerestrictions;

import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.lib.utilities.BoundingBox;

/**
 * A class that represents a zone that the robot is not allowed to enter.
 * Movement toward the zone is slowed and eventually blocked as the robot approaches.
 */
public class RestrictedZone implements ZoneRestriction {
    private final BoundingBox boundingBox;
    private final double minimumDistanceMeters, brakingZoneDistanceMeters;

    /**
     * Constructs a new RestrictedZone object.
     *
     * @param boundingBox               the bounding box of the restricted zone
     * @param minimumDistanceMeters     the distance from the zone boundary that the robot cannot cross
     * @param brakingZoneDistanceMeters the distance from the zone boundary at which braking begins
     */
    public RestrictedZone(BoundingBox boundingBox, double minimumDistanceMeters, double brakingZoneDistanceMeters) {
        this.boundingBox = boundingBox;
        this.minimumDistanceMeters = Math.max(0, minimumDistanceMeters);
        this.brakingZoneDistanceMeters = Math.max(this.minimumDistanceMeters, brakingZoneDistanceMeters);
    }

    @Override
    public BoundingBox getBoundingBox() {
        return boundingBox;
    }

    @Override
    public double getMinimumDistanceFromZoneBoundaryMeters() {
        return minimumDistanceMeters;
    }

    @Override
    public double getBrakingZoneDistanceMeters() {
        return brakingZoneDistanceMeters;
    }

    @Override
    public Translation2d applyRestriction(Translation2d targetTranslation, BoundingBox robotBoundingBox) {
        final double minimumDistanceToBoundaryMeters = robotBoundingBox.distanceTo(boundingBox);

        if (minimumDistanceToBoundaryMeters > brakingZoneDistanceMeters)
            return targetTranslation;

        return calculateRestrictedTranslation(targetTranslation, minimumDistanceToBoundaryMeters, robotBoundingBox);
    }

    /**
     * Scales down the component of the translation pointing toward the zone boundary.
     *
     * @param targetTranslation               the  robot's target translation
     * @param minimumDistanceToBoundaryMeters the current distance to the zone boundary from the closest point of the robot to the closest point on the boundary
     * @param robotBoundingBox                the robot's current bounding box
     * @return the restricted translation
     */
    private Translation2d calculateRestrictedTranslation(Translation2d targetTranslation, double minimumDistanceToBoundaryMeters, BoundingBox robotBoundingBox) {
        final Translation2d directionTowardBoundary = calculateDirectionTowardBoundary(robotBoundingBox);
        final double translationComponentTowardBoundary = targetTranslation.dot(directionTowardBoundary);

        if (translationComponentTowardBoundary <= 0)
            return targetTranslation;

        return applyBrakingScale(targetTranslation, directionTowardBoundary, translationComponentTowardBoundary, minimumDistanceToBoundaryMeters);
    }

    /**
     * Returns the direction from the robot toward the nearest zone boundary point.
     * If the robot is inside the zone such that the direction would be towards outside the zone,
     * the direction is flipped to prevent movement further into the zone.
     * Returns a zero vector if the robot is exactly on the boundary.
     *
     * @param robotBoundingBox the robot's current bounding box
     * @return the direction from the robot pointing toward the zone boundary
     */
    private Translation2d calculateDirectionTowardBoundary(BoundingBox robotBoundingBox) {
        final Translation2d robotCenter = robotBoundingBox.getCenter().getTranslation();
        Translation2d directionTowardBoundary = boundingBox.nearestPerimeterPoint(robotCenter).minus(robotCenter);

        if (boundingBox.contains(robotCenter))
            directionTowardBoundary = directionTowardBoundary.unaryMinus();

        if (directionTowardBoundary.getNorm() < 1e-6)
            return new Translation2d();

        return directionTowardBoundary.times(1 / directionTowardBoundary.getNorm());
    }

    /**
     * Splits the translation into a toward-boundary component and a perpendicular component,
     * scales the toward-boundary component by the braking scale, and recombines them.
     *
     * @param targetTranslation                the target translation to apply braking to
     * @param towardBoundaryComponentMagnitude the magnitude of the target translation in the toward-boundary direction
     * @param directionTowardBoundary         the direction of the robot pointing toward the boundary
     * @param minimumDistanceToBoundaryMeters  the current distance to the zone boundary from the closest point of the robot to the closest point on the boundary
     * @return the scaled translation
     */
    private Translation2d applyBrakingScale(Translation2d targetTranslation, Translation2d directionTowardBoundary, double towardBoundaryComponentMagnitude, double minimumDistanceToBoundaryMeters) {
        final Translation2d
                towardComponent = directionTowardBoundary.times(towardBoundaryComponentMagnitude),
                perpendicularComponent = targetTranslation.minus(towardComponent);
        final double brakingScale = calculateBrakingScale(minimumDistanceToBoundaryMeters);

        return perpendicularComponent.plus(towardComponent.times(brakingScale));
    }
}
