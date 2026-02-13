// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.swerve.Gyro;
import frc.robot.swerve.Swervedrive;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopDrive;

public class RobotContainer {

    Swervedrive m_swerve = new Swervedrive();
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    private final SendableChooser<Command> autoChooser;

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        m_swerve.setDefaultCommand(
            new TeleopDrive(m_swerve, () -> driveController.getLeftY(), () -> driveController.getLeftX(), () -> driveController.getRightX()) 
        );
        

        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Example Auto");
    }
}
