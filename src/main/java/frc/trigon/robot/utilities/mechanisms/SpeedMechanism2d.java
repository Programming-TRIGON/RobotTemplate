package frc.trigon.robot.utilities.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism2d object to display the current velocity and target velocity of a mechanism.
 */
public class SpeedMechanism2d {
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentVelocityLigament,
            currentVelocityTopArrowLigament,
            currentVelocityBottomArrowLigament,
            targetVelocityLigament,
            targetVelocityTopArrowLigament,
            targetVelocityBottomArrowLigament;
    private final double deadband;

    public SpeedMechanism2d(String key, double maximumDisplayableVelocity) {
        this(key, maximumDisplayableVelocity, 0.001);
    }

    public SpeedMechanism2d(String key, double maximumDisplayableVelocity, double deadband) {
        this.deadband = deadband;
        this.key = key;
        this.mechanism = new Mechanism2d(2 * maximumDisplayableVelocity, 2 * maximumDisplayableVelocity);
        MechanismRoot2d root = mechanism.getRoot("VelocityRoot", maximumDisplayableVelocity, maximumDisplayableVelocity);
        this.currentVelocityLigament = root.append(new MechanismLigament2d("ZCurrentVelocityLigament", 0, 0, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.BLUE));
        this.currentVelocityTopArrowLigament = currentVelocityLigament.append(new MechanismLigament2d("ZCurrentVelocityTopArrowLigament", MechanismConstants.ARROW_LENGTH_SCALE * maximumDisplayableVelocity, MechanismConstants.ZERO_TOP_ANGLE, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.BLUE));
        this.currentVelocityBottomArrowLigament = currentVelocityLigament.append(new MechanismLigament2d("ZCurrentVelocityBottomArrowLigament", MechanismConstants.ARROW_LENGTH_SCALE * maximumDisplayableVelocity, MechanismConstants.ZERO_BOTTOM_ANGLE, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.BLUE));

        this.targetVelocityLigament = root.append(new MechanismLigament2d("TargetVelocityLigament", 0, 0, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.GRAY));
        this.targetVelocityTopArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityTopArrowLigament", MechanismConstants.ARROW_LENGTH_SCALE * maximumDisplayableVelocity, MechanismConstants.ZERO_TOP_ANGLE, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.GRAY));
        this.targetVelocityBottomArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityBottomArrowLigament", MechanismConstants.ARROW_LENGTH_SCALE * maximumDisplayableVelocity, MechanismConstants.ZERO_BOTTOM_ANGLE, MechanismConstants.MECHANISM_LINE_WIDTH, MechanismConstants.GRAY));
    }

    /**
     * Updates the mechanism's velocity and target velocity and logs the Mechanism2d object.
     *
     * @param velocity       the current velocity
     * @param targetVelocity the target velocity
     */
    public void updateMechanism(double velocity, double targetVelocity) {
        setTargetVelocity(targetVelocity);
        updateMechanism(velocity);
    }

    /**
     * Updates the mechanism's velocity and logs the Mechanism2d object.
     *
     * @param velocity the current velocity
     */
    public void updateMechanism(double velocity) {
        setArrowAngle(velocity, currentVelocityTopArrowLigament, currentVelocityBottomArrowLigament);
        currentVelocityLigament.setLength(velocity);
        setCurrentLigamentColor(velocityToColor(velocity));
        Logger.recordOutput(key, mechanism);
    }

    /**
     * Sets the target velocity but doesn't log the Mechanism2d object.
     *
     * @param targetVelocity the target velocity
     */
    public void setTargetVelocity(double targetVelocity) {
        setArrowAngle(targetVelocity, targetVelocityTopArrowLigament, targetVelocityBottomArrowLigament);
        targetVelocityLigament.setLength(targetVelocity);
    }

    private void setCurrentLigamentColor(Color8Bit color) {
        currentVelocityLigament.setColor(color);
        currentVelocityTopArrowLigament.setColor(color);
        currentVelocityBottomArrowLigament.setColor(color);
    }

    private Color8Bit velocityToColor(double velocity) {
        if (velocity > deadband)
            return MechanismConstants.GREEN;
        else if (velocity < -deadband)
            return MechanismConstants.RED;
        return MechanismConstants.BLUE;
    }

    private void setArrowAngle(double velocity, MechanismLigament2d topLigament, MechanismLigament2d bottomLigament) {
        if (velocity > deadband) {
            topLigament.setAngle(MechanismConstants.POSITIVE_TOP_ANGLE);
            bottomLigament.setAngle(MechanismConstants.POSITIVE_BOTTOM_ANGLE);
        } else if (velocity < -deadband) {
            topLigament.setAngle(MechanismConstants.NEGATIVE_TOP_ANGLE);
            bottomLigament.setAngle(MechanismConstants.NEGATIVE_BOTTOM_ANGLE);
        } else {
            topLigament.setAngle(MechanismConstants.ZERO_TOP_ANGLE);
            bottomLigament.setAngle(MechanismConstants.ZERO_BOTTOM_ANGLE);
        }
    }
}