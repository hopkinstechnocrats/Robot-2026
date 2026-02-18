// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.swerve.Gyro;
import frc.robot.swerve.Swervedrive;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

    Swervedrive m_swerve = new Swervedrive();
    IntakeSubsystem m_intake = new IntakeSubsystem();
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    CommandXboxController operatorController = new CommandXboxController(Constants.k_operatorPort);

    public RobotContainer() {
        m_swerve.setDefaultCommand(
            new TeleopDrive(m_swerve, () -> driveController.getLeftY(), () -> driveController.getLeftX(), () -> driveController.getRightX()) 
        );
        
        m_intake.setDefaultCommand(
            new RunCommand(
                    () -> {
                    m_intake.intake(Constants.k_intakeBrakeSpeedRPS);
                  }, m_intake)
        );

        configureBindings();
    }

    private void configureBindings() {
        operatorController.a().whileTrue(IntakeCommands.intake(m_intake));
        operatorController.a().whileTrue(IntakeCommands.deployBob(m_intake)); //TODO check if works
        operatorController.b().whileTrue(IntakeCommands.outtake(m_intake));
        operatorController.y().whileTrue(IntakeCommands.deploy(m_intake));
        operatorController.x().whileTrue(IntakeCommands.undeploy(m_intake));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

