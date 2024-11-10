// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.factories.GeneralCommands;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    public static final Swerve SWERVE = new Swerve();
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator();
    public static LoggedDashboardChooser<Command> AUTO_CHOOSER;

    public RobotContainer() {
        configureBindings();
        buildAutoChooser();
    }

    /**
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return AUTO_CHOOSER.get();
    }

    private void configureBindings() {
        bindDefaultCommands();
        bindControllerCommands();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.SET_GYRO_HEADING_TO_SOLVE_PNP_HEADING_TRIGGER.onTrue(CommandConstants.SET_GYRO_HEADING_TO_SOLVE_PNP_HEADING_COMMAND);
        OperatorConstants.RESET_POSE_TO_AUTO_POSE_TRIGGER.onTrue(CommandConstants.RESET_POSE_TO_AUTO_POSE_COMMAND);
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.TOGGLE_FIELD_AND_SELF_RELATIVE_DRIVE_TRIGGER.onTrue(GeneralCommands.getToggleFieldAndSelfRelativeDriveCommand());
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(GeneralCommands.getToggleBrakeCommand());
    }

    private void buildAutoChooser() {
        AUTO_CHOOSER = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    }
}
