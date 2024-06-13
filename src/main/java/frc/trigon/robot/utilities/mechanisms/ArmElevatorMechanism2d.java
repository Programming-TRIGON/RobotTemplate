package frc.trigon.robot.utilities.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism2d object to display the current angle and target angle, and the current position and target position of an arm elevator.
 */
public class ArmElevatorMechanism2d {
    private static final Color8Bit
            GRAY = new Color8Bit(Color.kGray),
            BLUE = new Color8Bit(Color.kBlue);
    private static final double MECHANISM_LINE_WIDTH = 5;
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentPositionLigament,
            targetPositionLigament;

    public ArmElevatorMechanism2d(String key, double maximumDisplayablePosition) {
        this(key, maximumDisplayablePosition, BLUE);
    }

    public ArmElevatorMechanism2d(String key, double maximumDisplayablePosition, Color8Bit mechanismColor) {
        this.key = key;
        this.mechanism = new Mechanism2d(2 * maximumDisplayablePosition, 2 * maximumDisplayablePosition);
        MechanismRoot2d root = mechanism.getRoot("CurrentPositionRoot", maximumDisplayablePosition, maximumDisplayablePosition);
        this.currentPositionLigament = root.append(new MechanismLigament2d("CurrentPositionLigament", 0, 0, MECHANISM_LINE_WIDTH, mechanismColor));
        this.targetPositionLigament = root.append(new MechanismLigament2d("TargetPositionLigament", 0, 0, MECHANISM_LINE_WIDTH, GRAY));
    }

    public void updateMechanism(double currentPosition, double targetPosition, Rotation2d currentAngle, Rotation2d targetAngle) {
        updateMechanism(currentPosition, targetPosition, currentAngle.getDegrees(), targetAngle.getDegrees());
    }

    public void updateMechanism(double currentPosition, double targetPosition, double currentAngleDegrees, double targetAngleDegrees) {
        setTargetPosition(targetPosition);
        setTargetAngle(targetAngleDegrees);
        updateMechanism(currentPosition, currentAngleDegrees);
    }

    public void updateMechanism(double currentPosition, Rotation2d currentAngle) {
        updateMechanism(currentPosition, currentAngle.getDegrees());
    }

    public void updateMechanism(double currentPosition, double currentAngleDegrees) {
        updateCurrentPosition(currentPosition);
        updateCurrentAngle(currentAngleDegrees);
        Logger.recordOutput(key, mechanism);
    }

    public void updateCurrentPosition(double currentPosition) {
        currentPositionLigament.setLength(currentPosition);
    }

    public void updateCurrentAngle(Rotation2d currentAngle) {
        updateCurrentAngle(currentAngle.getDegrees());
    }

    public void updateCurrentAngle(double currentAngleDegrees) {
        currentPositionLigament.setAngle(currentAngleDegrees);
    }

    public void setTargetPosition(double targetPosition) {
        targetPositionLigament.setLength(targetPosition);
    }

    public void setTargetAngle(Rotation2d targetAngle) {
        setTargetAngle(targetAngle.getDegrees());
    }

    public void setTargetAngle(double targetAngleDegrees) {
        targetPositionLigament.setAngle(targetAngleDegrees);
    }
}
