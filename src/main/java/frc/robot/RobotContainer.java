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
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.commands.FeederCommands;


public class RobotContainer {

    Swervedrive m_swerve = new Swervedrive();
    FeederSubsystem FeederSubsystem = new FeederSubsystem();
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    CommandXboxController operatorController = new CommandXboxController(Constants.ControlConstants.k_operatorXboxControllerPort);

    public RobotContainer() {
        FeederSubsystem.setDefaultCommand(
        new RunCommand(
                    () -> {
                    FeederSubsystem.feederBrake();
                  }, FeederSubsystem
      ));

        m_swerve.setDefaultCommand(
            new TeleopDrive(m_swerve, () -> driveController.getLeftY(), () -> driveController.getLeftX(), () -> driveController.getRightX()) 
        );
        

        configureBindings();
    }

    private void configureBindings() {
        operatorController.povRight().onTrue(FeederCommands.feeder(FeederSubsystem));
        operatorController.povLeft().onTrue(FeederCommands.unfeeder(FeederSubsystem)); 
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
