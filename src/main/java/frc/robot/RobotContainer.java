// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

public class RobotContainer {
    
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    CommandXboxController operatorController = new CommandXboxController(Constants.ControlConstants.k_operatorXboxControllerPort);
    private final HopperSubsystem hopperSubsystem = new HopperSubsystem();
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
    
    
      Command stopEverything(){
        return Commands.sequence(
        IntakeCommands.setIntakeSpeedOnce(intakeSubsystem, 0),
        LauncherCommands.launcherBreakOnce(launcherSubsystem),
        HopperCommands.setHopperSpeedOnce(hopperSubsystem, 0),
        FeederCommands.setFeederSpeedOnce(feederSubsystem, 0));
      };

      Command LaunchFuel(){
        return Commands.sequence(
          IntakeCommands.launchingOnce(intakeSubsystem),
          LauncherCommands.setlaunchSpeedOnce(launcherSubsystem, Constants.LauncherConstants.k_launchSpeedRPS),
          //LauncherCommands.setlaunchSpeedOnce(launcherSubsystem, Constants.IntakeConstants.k_intakeSpeedRPS),
          Commands.waitSeconds(0.3),//TODO: update the delays between launcher/feeder/hopper.
          FeederCommands.setFeederSpeedOnce(feederSubsystem, Constants.FeederConstants.k_feederSpeedRPS),
          Commands.waitSeconds(0.1),
          HopperCommands.setHopperSpeedOnce(hopperSubsystem, Constants.HopperConstants.k_hopperSpeedRPS)
        );
     }

    Autos auto = new Autos();
    Swervedrive m_swerve = new Swervedrive();
    
    public void restoreDefaults(){
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
    }

    public RobotContainer() {

        configureButtonBindings();
    } 

    public Command getAutonomousCommand() {
      //The entirity of our auto.
        return Commands.sequence(
          LaunchFuel(),
          Commands.waitSeconds(5),          
          stopEverything(),
          TeleopDrive.autoExecute(0,1,0,m_swerve),
          Commands.waitSeconds(10),
          TeleopDrive.autoExecute(0,0,0,m_swerve)
          );
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
      operatorController.rightTrigger().whileTrue(LauncherCommands.launcher(launcherSubsystem).alongWith(IntakeCommands.launching(intakeSubsystem)).withTimeout(0.5)
        .andThen(FeederCommands.feeder(feederSubsystem).alongWith(HopperCommands.hopper(hopperSubsystem).alongWith(LauncherCommands.launcher(launcherSubsystem)))));
      operatorController.povLeft().whileTrue(LauncherCommands.inverseLauncher(launcherSubsystem));
    }
}
