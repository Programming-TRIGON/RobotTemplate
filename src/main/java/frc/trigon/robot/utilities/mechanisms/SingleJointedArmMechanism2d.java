package frc.trigon.robot.utilities.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism2d object to display the current angle and the target angle of a single jointed arm.
 */
public class SingleJointedArmMechanism2d {
    private static final Color8Bit
            GRAY = new Color8Bit(Color.kGray),
            BLUE = new Color8Bit(Color.kBlue);
    private static final double
            MECHANISM_LINE_WIDTH = 5,
            MECHANISM_LINE_LENGTH = 10;
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentPositionLigament,
            targetPositionLigament;

    public SingleJointedArmMechanism2d(String key) {
        this(key, BLUE);
    }

    public SingleJointedArmMechanism2d(String key, Color8Bit mechanismColor) {
        this.key = key;
        this.mechanism = new Mechanism2d(2 * MECHANISM_LINE_LENGTH, 2 * MECHANISM_LINE_LENGTH);
        MechanismRoot2d root = mechanism.getRoot("AngleRoot", MECHANISM_LINE_LENGTH, MECHANISM_LINE_LENGTH);
        this.currentPositionLigament = root.append(new MechanismLigament2d("ZCurrentPositionLigament", MECHANISM_LINE_LENGTH, 0, MECHANISM_LINE_WIDTH, mechanismColor));
        this.targetPositionLigament = root.append(new MechanismLigament2d("TargetPositionLigament", MECHANISM_LINE_LENGTH, 0, MECHANISM_LINE_WIDTH, GRAY));
    }

    public void updateMechanism(Rotation2d currentAngle, Rotation2d targetAngle) {
        updateMechanism(currentAngle.getDegrees(), targetAngle.getDegrees());
    }

    public void updateMechanism(double currentAngleDegrees, double targetAngle) {
        setTargetAngle(targetAngle);
        updateMechanism(currentAngleDegrees);
    }

    public void updateMechanism(Rotation2d currentAngle) {
        updateMechanism(currentAngle.getDegrees());
    }

    public void updateMechanism(double currentAngle) {
        currentPositionLigament.setAngle(currentAngle);
        Logger.recordOutput(key, mechanism);
    }

    public void setTargetAngle(double targetAngle) {
        targetPositionLigament.setAngle(targetAngle);
    }
}
