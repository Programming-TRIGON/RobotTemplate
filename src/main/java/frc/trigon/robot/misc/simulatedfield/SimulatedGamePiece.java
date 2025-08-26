package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

public class SimulatedGamePiece {
    protected SimulatedGamePieceConstants.GamePieceType gamePieceType;
    protected boolean isScored = false;
    private Pose3d fieldRelativePose;
    private Pose3d poseAtRelease;
    private Translation3d velocityAtRelease = new Translation3d();
    private double timestampAtRelease = 0;
    private boolean isTouchingGround = true;

    public SimulatedGamePiece(Pose3d startingPose, SimulatedGamePieceConstants.GamePieceType gamePieceType) {
        fieldRelativePose = startingPose;
        this.gamePieceType = gamePieceType;
    }

    public void updatePeriodically(boolean isHeld) {
        if (!isHeld)
            checkScored();
        if (!isScored && !isTouchingGround)
            applyGravity();
    }

    /**
     * Releases the game piece from the robot.
     *
     * @param fieldRelativeReleaseVelocity the velocity that the object is released at, relative to the field
     */
    public void release(Translation3d fieldRelativeReleaseVelocity) {
        velocityAtRelease = fieldRelativeReleaseVelocity;
        poseAtRelease = fieldRelativePose;
        timestampAtRelease = Timer.getTimestamp();

        updateIsTouchingGround();
    }

    public void updatePose(Pose3d fieldRelativePose) {
        this.fieldRelativePose = fieldRelativePose;
    }

    public Pose3d getPose() {
        return fieldRelativePose;
    }

    public double getDistanceFromPoseMeters(Pose3d pose) {
        return fieldRelativePose.minus(pose).getTranslation().getNorm();
    }

    public boolean isScored() {
        return isScored;
    }

    private void applyGravity() {
        if (poseAtRelease == null)
            return;
        final double timeSinceEject = Timer.getTimestamp() - timestampAtRelease;
        this.fieldRelativePose = new Pose3d(poseAtRelease.getTranslation(), new Rotation3d()).transformBy(calculateVelocityPoseTransform(timeSinceEject));

        updateIsTouchingGround();
    }

    private Transform3d calculateVelocityPoseTransform(double elapsedTime) {
        return new Transform3d(
                velocityAtRelease.getX() * elapsedTime,
                velocityAtRelease.getY() * elapsedTime,
                velocityAtRelease.getZ() * elapsedTime - ((SimulatedGamePieceConstants.G_FORCE / 2) * elapsedTime * elapsedTime),
                poseAtRelease.getRotation()
        );
    }

    private void checkScored() {
        if (!isScored)
            SimulationScoringHandler.checkGamePieceScored(this);
    }

    private void updateIsTouchingGround() {
        if (fieldRelativePose.getZ() < gamePieceType.originPointHeightOffGroundMeters) {
            isTouchingGround = true;
            velocityAtRelease = new Translation3d();

            final Translation3d fieldRelativeTranslation = new Translation3d(
                    fieldRelativePose.getTranslation().getX(),
                    fieldRelativePose.getTranslation().getY(),
                    gamePieceType.originPointHeightOffGroundMeters
            );
            final Rotation3d fieldRelativeRotation = new Rotation3d(
                    fieldRelativePose.getRotation().getX(),
                    0,
                    fieldRelativePose.getRotation().getZ()
            );
            fieldRelativePose = new Pose3d(fieldRelativeTranslation, fieldRelativeRotation);
            return;
        }
        isTouchingGround = false;
    }
}
