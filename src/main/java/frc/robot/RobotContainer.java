// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.swerve.Gyro;
import frc.robot.swerve.Swervedrive;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.commands.HopperCommands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.commands.AutonomusCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.LauncherCommands;

public class RobotContainer {
    
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);
    CommandXboxController operatorController = new CommandXboxController(Constants.ControlConstants.k_operatorXboxControllerPort);
    private final HopperSubsystem hopperSubsystem = new HopperSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
        RobotConfig pathPlannerConfig;


    private final SendableChooser<Command> autoChooser;


    //Command exampleAutoCommand = new PathPlannerAuto("SmallSquareAuto");

    public void configureNamedCommands() {
      NamedCommands.registerCommand("runIntake", IntakeCommands.intake(intakeSubsystem));
      NamedCommands.registerCommand("brakeIntake", IntakeCommands.brakeIntake(intakeSubsystem));
      NamedCommands.registerCommand("runHopper", HopperCommands.hopper(hopperSubsystem));
      NamedCommands.registerCommand("brakeHopper", HopperCommands.brake(hopperSubsystem));
      NamedCommands.registerCommand("runFeeder", FeederCommands.feeder(feederSubsystem));
      NamedCommands.registerCommand("brakeFeeder", FeederCommands.brakeFeeder(feederSubsystem));
      NamedCommands.registerCommand("runLauncher", LauncherCommands.launcher(launcherSubsystem));
      NamedCommands.registerCommand("brakeLauncher", LauncherCommands.launcherBreak(launcherSubsystem)); 
    }


    
    Swervedrive m_swerve = new Swervedrive();

    public RobotContainer() {
        feederSubsystem.setDefaultCommand(FeederCommands.brakeFeeder(feederSubsystem));

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

        configureNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("MoveForwardAuto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    } 

    public Command getAutonomousCommand() {
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
