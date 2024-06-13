package frc.trigon.robot.utilities.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism2d object to display the current angle and the target angle of a double jointed arm.
 */
public class DoubleJointedArmMechanism2d {
    private static final Color8Bit
            GRAY = new Color8Bit(Color.kGray),
            BLUE = new Color8Bit(Color.kBlue);
    private static final double
            MECHANISM_LINE_WIDTH = 5,
            MECHANISM_LINE_LENGTH = 5;
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentPositionFirstLigament,
            currentPositionSecondLigament,
            targetPositionFirstLigament,
            targetPositionSecondLigament;

    public DoubleJointedArmMechanism2d(String key) {
        this(key, BLUE);
    }

    private DoubleJointedArmMechanism2d(String key, Color8Bit mechanismColor) {
        this.key = key;
        this.mechanism = new Mechanism2d(4 * MECHANISM_LINE_LENGTH, 4 * MECHANISM_LINE_LENGTH);
        MechanismRoot2d root = mechanism.getRoot("AngleRoot", 2 * MECHANISM_LINE_LENGTH, 2 * MECHANISM_LINE_LENGTH);
        this.currentPositionFirstLigament = root.append(new MechanismLigament2d("CurrentPositionFirstLigament", MECHANISM_LINE_LENGTH, 0, MECHANISM_LINE_WIDTH, mechanismColor));
        this.currentPositionSecondLigament = currentPositionFirstLigament.append(new MechanismLigament2d("CurrentPositionSecondLigament", MECHANISM_LINE_LENGTH, 0, MECHANISM_LINE_WIDTH, mechanismColor));
        this.targetPositionFirstLigament = root.append(new MechanismLigament2d("TargetPositionFirstLigament", MECHANISM_LINE_LENGTH, 0, MECHANISM_LINE_WIDTH, GRAY));
        this.targetPositionSecondLigament = targetPositionFirstLigament.append(new MechanismLigament2d("TargetPositionSecondLigament", MECHANISM_LINE_LENGTH, 0, MECHANISM_LINE_WIDTH, GRAY));
    }

    public void updateMechanism(Rotation2d firstJointCurrentAngle, Rotation2d firstJointTargetAngle, Rotation2d secondJointCurrentAngle, Rotation2d secondJointTargetAngle) {
        updateMechanism(firstJointCurrentAngle.getDegrees(), firstJointTargetAngle.getDegrees(), secondJointCurrentAngle.getDegrees(), secondJointTargetAngle.getDegrees());
    }

    public void updateMechanism(double firstJointCurrentAngle, double firstJointTargetAngle, double secondJointCurrentAngle, double secondJointTargetAngle) {
        setTargetAngle(firstJointTargetAngle, secondJointTargetAngle);
        updateMechanism(firstJointCurrentAngle, secondJointCurrentAngle);
    }

    public void updateMechanism(Rotation2d firstJointCurrentAngle, Rotation2d secondJointCurrentAngle) {
        updateMechanism(firstJointCurrentAngle.getDegrees(), secondJointCurrentAngle.getDegrees());
    }

    public void updateMechanism(double firstJointCurrentAngle, double secondJointCurrentAngle) {
        updateFirstJoint(firstJointCurrentAngle);
        updateSecondJoint(secondJointCurrentAngle);
        Logger.recordOutput(key, mechanism);
    }

    public void setTargetAngle(Rotation2d firstJointTargetAngle, Rotation2d secondJointTargetAngle) {
        setTargetAngle(firstJointTargetAngle.getDegrees(), secondJointTargetAngle.getDegrees());
    }

    public void setTargetAngle(double firstJointTargetAngle, double secondJointTargetAngle) {
        setFirstJointTargetAngle(firstJointTargetAngle);
        setSecondJointTargetAngle(secondJointTargetAngle);
    }

    public void updateFirstJoint(Rotation2d firstJointCurrentAngle, Rotation2d firstJointTargetAngle) {
        updateFirstJoint(firstJointCurrentAngle.getDegrees(), firstJointTargetAngle.getDegrees());
    }

    public void updateFirstJoint(double firstJointCurrentAngle, double firstJointTargetAngle) {
        updateFirstJoint(firstJointCurrentAngle);
        setFirstJointTargetAngle(firstJointTargetAngle);
    }

    public void updateFirstJoint(Rotation2d firstJointCurrentAngle) {
        updateFirstJoint(firstJointCurrentAngle.getDegrees());
    }

    public void updateFirstJoint(double firstJointCurrentAngle) {
        currentPositionFirstLigament.setAngle(firstJointCurrentAngle);
    }

    public void setFirstJointTargetAngle(Rotation2d firstJointTargetAngle) {
        setFirstJointTargetAngle(firstJointTargetAngle.getDegrees());
    }

    public void setFirstJointTargetAngle(double firstJointTargetAngle) {
        targetPositionFirstLigament.setAngle(firstJointTargetAngle);
    }

    public void updateSecondJoint(Rotation2d secondJointCurrentAngle, Rotation2d secondJointTargetAngle) {
        updateSecondJoint(secondJointCurrentAngle.getDegrees(), secondJointTargetAngle.getDegrees());
    }

    public void updateSecondJoint(double secondJointCurrentAngle, double secondJointTargetAngle) {
        setSecondJointTargetAngle(secondJointTargetAngle);
        updateSecondJoint(secondJointCurrentAngle);
    }

    private void updateSecondJoint(Rotation2d secondJointCurrentAngle) {
        updateSecondJoint(secondJointCurrentAngle.getDegrees());
    }

    private void updateSecondJoint(double secondJointCurrentAngle) {
        currentPositionSecondLigament.setAngle(secondJointCurrentAngle);
    }

    public void setSecondJointTargetAngle(Rotation2d secondJointTargetAngle) {
        setSecondJointTargetAngle(secondJointTargetAngle.getDegrees());
    }

    public void setSecondJointTargetAngle(double secondJointTargetAngle) {
        targetPositionSecondLigament.setAngle(secondJointTargetAngle);
    }
}
