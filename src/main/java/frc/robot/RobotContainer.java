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

public class RobotContainer {
    
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    CommandXboxController operatorController = new CommandXboxController(Constants.ControlConstants.k_operatorXboxControllerPort);
    EmptySubsystem emptySubsystem;
    
    public RobotContainer() {
      emptySubsystem = new EmptySubsystem();
        /*feederSubsystem.setDefaultCommand(FeederCommands.brakeFeeder(feederSubsystem));


        launcherSubsystem.setDefaultCommand(LauncherCommands.launcherBreak(launcherSubsystem));
        
		    intakeSubsystem.setDefaultCommand(
            new RunCommand(
                    () -> {
                    intakeSubsystem.intakeBrake();
                  }, intakeSubsystem
        ));

        hopperSubsystem.setDefaultCommand(HopperCommands.brake(hopperSubsystem));*/
        configureButtonBindings();
    } 

    private void configureButtonBindings() {
      //operatorController.a().whileTrue(IntakeCommands.intake(intakeSubsystem));
      //operatorController.b().whileTrue(IntakeCommands.outtake(intakeSubsystem));
      //operatorController.povUp().whileTrue(IntakeCommands.deploy(intakeSubsystem));
      //operatorController.povRight().whileTrue(IntakeCommands.undeploy(intakeSubsystem));
      
      //operatorController.x().whileTrue(HopperCommands.reverseHopper(hopperSubsystem));
      //operatorController.y().whileTrue(FeederCommands.unfeeder(feederSubsystem)); 
      /*operatorController.rightTrigger().whileTrue(LauncherCommands.launcher(launcherSubsystem)
              .alongWith(FeederCommands.feeder(feederSubsystem))
              .alongWith(HopperCommands.hopper(hopperSubsystem)));
      operatorController.povLeft().whileTrue(LauncherCommands.inverseLauncher(launcherSubsystem));
      */
    }
}
