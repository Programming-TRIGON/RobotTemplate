package frc.trigon.robot.utilities.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism2d object to display the current position and target position of an elevator.
 */
public class ElevatorMechanism2d {
    private static final double MECHANISM_STARTING_ANGLE = 90;
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentPositionLigament,
            targetPositionLigament;

    public ElevatorMechanism2d(String key, double maximumDisplayablePosition) {
        this(key, maximumDisplayablePosition, MechanismConstants.BLUE);
    }

    public ElevatorMechanism2d(String key, double maximumDisplayablePosition, Color8Bit mechanismColor) {
        this.key = key;
        this.mechanism = new Mechanism2d(2 * maximumDisplayablePosition, 2 * maximumDisplayablePosition);

        MechanismRoot2d currentPositionRoot = mechanism.getRoot("CurrentPositionRoot", 0.5 * maximumDisplayablePosition, 0);
        MechanismRoot2d targetPositionRoot = mechanism.getRoot("TargetPositionRoot", 1.5 * maximumDisplayablePosition, 0);
        this.currentPositionLigament = currentPositionRoot.append(new MechanismLigament2d("CurrentPositionLigament", 0, MECHANISM_STARTING_ANGLE, MechanismConstants.MECHANISM_LINE_WIDTH, mechanismColor));
        this.targetPositionLigament = targetPositionRoot.append(new MechanismLigament2d("TargetPositionLigament", 0, MECHANISM_STARTING_ANGLE, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.GRAY));
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
        currentPositionLigament.setLength(currentPosition);
        Logger.recordOutput(key, mechanism);
    }

    /**
     * Sets the target position of the mechanism.
     *
     * @param targetPosition the target position
     */
    public void setTargetPosition(double targetPosition) {
        targetPositionLigament.setLength(targetPosition);
    }
}
