package frc.trigon.robot.utilities.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism2d object to display the current position and target position of an elevator.
 */
public class ElevatorMechanism2d {
    private static final Color8Bit
            GRAY = new Color8Bit(Color.kGray),
            BLUE = new Color8Bit(Color.kBlue);
    private static final double MECHANISM_LINE_WIDTH = 5;
    private static final double MECHANISM_STARTING_ANGLE = 90;
    private final Color8Bit mechanismColor;
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentPositionLigament,
            targetPositionLigament;

    public ElevatorMechanism2d(String key, double maximumDisplayablePosition) {
        this(key, maximumDisplayablePosition, BLUE);
    }

    public ElevatorMechanism2d(String key, double maximumDisplayablePosition, Color8Bit mechanismColor) {
        this.key = key;
        this.mechanismColor = mechanismColor;
        this.mechanism = new Mechanism2d(2 * maximumDisplayablePosition, 2 * maximumDisplayablePosition);

        MechanismRoot2d currentPositionRoot = mechanism.getRoot("CurrentPositionRoot", 0.5 * maximumDisplayablePosition, 0);
        MechanismRoot2d targetPositionRoot = mechanism.getRoot("TargetPositionRoot", 1.5 * maximumDisplayablePosition, 0);
        this.currentPositionLigament = currentPositionRoot.append(new MechanismLigament2d("CurrentPositionLigament", 0, MECHANISM_STARTING_ANGLE, MECHANISM_LINE_WIDTH, mechanismColor));
        this.targetPositionLigament = targetPositionRoot.append(new MechanismLigament2d("TargetPositionLigament", 0, MECHANISM_STARTING_ANGLE, MECHANISM_LINE_WIDTH, GRAY));
    }

    public void updateMechanism(double currentPosition, double targetPosition) {
        setTargetPosition(targetPosition);
        updateMechanism(currentPosition);
    }

    public void updateMechanism(double currentPosition) {
        currentPositionLigament.setLength(currentPosition);
        Logger.recordOutput(key, mechanism);
    }

    public void setTargetPosition(double targetPosition) {
        targetPositionLigament.setLength(targetPosition);
    }
}
