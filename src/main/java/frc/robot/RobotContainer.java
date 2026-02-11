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
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TurretCommands;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

    Swervedrive m_swerve = new Swervedrive();
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    CommandXboxController operatorController = new CommandXboxController(Constants.ControlConstants.k_operatorPort);
    TurretSubsystem turretSubsystem = new TurretSubsystem();

    public RobotContainer() {
        m_swerve.setDefaultCommand(
            new TeleopDrive(m_swerve, () -> driveController.getLeftY(), () -> driveController.getLeftX(), () -> driveController.getRightX()) 
        );
        

        configureBindings();
        operatorController.rightBumper().whileTrue(TurretCommands.turret(turretSubsystem));
        operatorController.leftBumper().whileTrue(TurretCommands.reverseTurret(turretSubsystem));
    }

    private void configureBindings() {
         turretSubsystem.setDefaultCommand(
          new RunCommand(
                  () -> {
                  turretSubsystem.turret(Constants.TurretConstants.k_turretBrakeSpeedRPS);
                }, turretSubsystem)
      );
        
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
