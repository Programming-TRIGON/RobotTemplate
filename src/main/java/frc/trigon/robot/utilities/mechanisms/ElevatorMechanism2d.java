package frc.trigon.robot.utilities.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ElevatorMechanism2d {
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentPositionLigament,
            targetPositionLigament;
    private final double minimumLength;

    /**
     * Constructs an ElevatorMechanism2d object.
     *
     * @param key            the key of the mechanism
     * @param maximumLength  the maximum length of the elevator
     * @param minimumLength  the minimum length of the elevator
     * @param mechanismColor the color of the mechanism
     */
    public ElevatorMechanism2d(String key, double maximumLength, double minimumLength, Color8Bit mechanismColor) {
        this.key = key;
        this.minimumLength = minimumLength;
        this.mechanism = new Mechanism2d(maximumLength, maximumLength);

        MechanismRoot2d currentPositionRoot = mechanism.getRoot("ZCurrentPositionRoot", 0.5 * maximumLength, 0);
        MechanismRoot2d targetPositionRoot = mechanism.getRoot("TargetPositionRoot", 0.5 * maximumLength, 0);
        this.currentPositionLigament = currentPositionRoot.append(new MechanismLigament2d("ZCurrentPositionLigament", minimumLength, MechanismConstants.ELEVATOR_MECHANISM_STARTING_ANGLE, MechanismConstants.MECHANISM_LINE_WIDTH, mechanismColor));
        this.targetPositionLigament = targetPositionRoot.append(new MechanismLigament2d("TargetPositionLigament", maximumLength, MechanismConstants.ELEVATOR_MECHANISM_STARTING_ANGLE, MechanismConstants.TARGET_POSITION_LIGAMENT_WIDTH, MechanismConstants.GRAY));
    }

    /**
     * Updates the mechanism's position and target position, then logs the Mechanism2d object.
     *
     * @param currentPosition the current position
     * @param targetPosition  the target position
     */
    public void updateMechanism(double currentPosition, double targetPosition) {
        setTargetPosition(targetPosition);
        updateMechanism(currentPosition);
    }

    /**
     * Updates the mechanism's position, then logs the Mechanism2d object.
     *
     * @param currentPosition the current position
     */
    public void updateMechanism(double currentPosition) {
        currentPositionLigament.setLength(currentPosition + minimumLength);
        Logger.recordOutput(key, mechanism);
    }

    /**
     * Sets the target position of the mechanism, but doesn't log the Mechanism2d object.
     *
     * @param targetPosition the target position
     */
    public void setTargetPosition(double targetPosition) {
        targetPositionLigament.setLength(targetPosition + minimumLength);
    }
}