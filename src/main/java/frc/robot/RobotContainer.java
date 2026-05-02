// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.swerve.Swervedrive;
import frc.robot.autos.Autos;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.commands.HopperCommands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.commands.FeederCommands;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.LauncherCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;

public class RobotContainer {
    
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    CommandXboxController operatorController = new CommandXboxController(Constants.ControlConstants.k_operatorXboxControllerPort);
    private final HopperSubsystem hopperSubsystem = new HopperSubsystem();
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();

    SendableChooser<Command> autoChooser;
    RobotConfig roboConfig;

    Autos auto = new Autos();
    Swervedrive m_swerve = new Swervedrive();
    
    public void setupAutos(){
      NamedCommands.registerCommand("runLauncher", LauncherCommands.launcher(launcherSubsystem));
      NamedCommands.registerCommand("brakeLauncher", LauncherCommands.launcherBreak(launcherSubsystem)); 
      
      autoChooser = AutoBuilder.buildAutoChooser("LauncherTest");
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public RobotContainer() {
        feederSubsystem.setDefaultCommand(FeederCommands.brakeFeeder(feederSubsystem));

        m_chooser.setDefaultOption("1 second", auto.oneSecond(m_swerve, 4)); //spped x & y is meters/second
        m_swerve.setDefaultCommand(
            new TeleopDrive(m_swerve, () -> driveController.getLeftY(), () -> driveController.getLeftX(), () -> driveController.getRightX(),
                ()->driveController.getRightTriggerAxis(), () -> driveController.getLeftTriggerAxis()) 
        );


        launcherSubsystem.setDefaultCommand(LauncherCommands.launcherBreak(launcherSubsystem));
        
		    intakeSubsystem.setDefaultCommand(
            new RunCommand(
                    () -> {
                    intakeSubsystem.intakeBrake();
                  }, intakeSubsystem
        ));

        hopperSubsystem.setDefaultCommand(HopperCommands.brake(hopperSubsystem));

        configureButtonBindings();
    } 

    public Command getAutonomousCommand() {
      setupAutos();
      return autoChooser.getSelected();
    }

    private void configureButtonBindings() {
      operatorController.a().whileTrue(IntakeCommands.intake(intakeSubsystem));
      operatorController.b().whileTrue(IntakeCommands.outtake(intakeSubsystem));
      operatorController.povUp().whileTrue(IntakeCommands.up(intakeSubsystem));
      operatorController.povRight().whileTrue(IntakeCommands.down(intakeSubsystem));
      driveController.a().onTrue(Commands.runOnce(
        () -> m_swerve.resetHeading(),
        m_swerve));
      
      operatorController.x().whileTrue(HopperCommands.reverseHopper(hopperSubsystem));
      operatorController.y().whileTrue(FeederCommands.unfeeder(feederSubsystem)); 
      operatorController.rightTrigger().whileTrue(LauncherCommands.launcher(launcherSubsystem).withTimeout(0.5)
        .andThen(FeederCommands.feeder(feederSubsystem).alongWith(HopperCommands.hopper(hopperSubsystem).alongWith(LauncherCommands.launcher(launcherSubsystem)))));
      operatorController.povLeft().whileTrue(LauncherCommands.inverseLauncher(launcherSubsystem));
    }
}
