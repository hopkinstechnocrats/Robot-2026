// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.swerve.Gyro;
import frc.robot.autos.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.commands.HopperCommands;
frc.robot.subsystems.IntakeSubsystem;
import frc.robot.swerve.Swervedrive;
import frc.robot.commands.IntakeCommands;

public class RobotContainer {

    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    private final CommandXboxController operatorController = new CommandXboxController(Constants.ControlConstants.operatorXboxControllerPort);

    private final HopperSubsystem hopperSubsystem = new HopperSubsystem();
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    
    Autos auto = new Autos();
    Swervedrive m_swerve = new Swervedrive();
    
    
    public RobotContainer() {
        m_chooser.setDefaultOption("forward auto", auto.complexAuto(m_swerve, 2)); //spped x & y is meters/second
        m_swerve.setDefaultCommand(
            new TeleopDrive(m_swerve, () -> driveController.getLeftY(), () -> driveController.getLeftX(), () -> driveController.getRightX()) 
        );

		    intakeSubsystem.setDefaultCommand(
            new RunCommand(
                    () -> {
                    intakeSubsystem.intakeBrake();
                  }, intakeSubsystem
        ));

        configureButtonBindings();
    } 


    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    private void configureButtonBindings() {
      operatorController.a().whileTrue(IntakeCommands.intake(intakeSubsystem));
      operatorController.b().whileTrue(IntakeCommands.outtake(intakeSubsystem));
      operatorController.y().whileTrue(IntakeCommands.deploy(intakeSubsystem));
      operatorController.x().whileTrue(IntakeCommands.undeploy(intakeSubsystem));
      operatorController.a().whileTrue(HopperCommands.hopper(hopperSubsystem));
      operatorController.b().whileTrue(HopperCommands.reverseHopper(hopperSubsystem));
    }
}
