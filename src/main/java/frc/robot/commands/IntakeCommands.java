package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommands extends Command {
       
    public static Command intake(IntakeSubsystem intake) {
        return Commands.run(
            () -> {
                intake.intake(0.5);
                intake.intakeDeploy(0.005);
            },
        intake);
    }

    public static Command outtake(IntakeSubsystem intake) {
        return Commands.run(
            () -> {
                intake.intake(-0.5);
            },
        intake);
    }
    public static Command setIntakeSpeedOnce(IntakeSubsystem intake,double speed) {
        return Commands.runOnce(
            () -> {
                intake.intake(speed);
            },
        intake);
    }

    public static Command deploy(IntakeSubsystem intake) {
        return Commands.run(
            () -> {
                intake.intakeDeploy(Constants.IntakeConstants.k_intakeSetpointDeploy);
            },
        intake);
    }
/*
    public static Command deployBob(IntakeSubsystem deployBob) {
        return Commands.run(
            () -> {
                deployBob.intakeBob();
            },
        deployBob);
    }
*/
    public static Command undeploy(IntakeSubsystem intake) {
        return Commands.run(
            () -> {
                intake.intakeDeploy(Constants.IntakeConstants.k_intakeSetpointRetract);
            },
        intake);
    }

    public static Command up(IntakeSubsystem intake){
        return Commands.run(
            () -> {
                intake.intakeDeploy(0.14);
            },
            intake);
    }

    public static Command launching(IntakeSubsystem intake){
        return Commands.run(
            () -> {
                intake.intakeDeploy(0.03);
                intake.intake(0.3);
            },
            intake);
    }

    
    public static Command launchingOnce(IntakeSubsystem intake){
        return Commands.runOnce(
            () -> {
                intake.intakeDeploy(0.03);
                intake.intake(0.3);
            },
            intake);
    }

    public static Command down(IntakeSubsystem intake){
        return Commands.run(
            () -> {
                intake.intakeDeploy(0.01);
            },
            intake);
    }
}
