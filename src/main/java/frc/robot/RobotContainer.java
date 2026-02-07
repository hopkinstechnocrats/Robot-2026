// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.swerve.Gyro;
import frc.robot.swerve.Swervedrive;
import frc.robot.autos.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopDrive;

public class RobotContainer {

    Swervedrive m_swerve = new Swervedrive();
    Autos auto = new Autos();
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        m_chooser.setDefaultOption("forward auto", auto.complexAuto(m_swerve, 2)); //spped x & y is meters/second
        m_swerve.setDefaultCommand(
            new TeleopDrive(m_swerve, () -> driveController.getLeftY(), () -> driveController.getLeftX(), () -> driveController.getRightX()) 
        );
        

        configureBindings();
    }

    private void configureBindings() {
        
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
