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
            },
        intake);
    }

    public static Command setIntakeSpeed(IntakeSubsystem intake,double speed) {
        return Commands.run(
            () -> {
                intake.intake(speed);
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
                intake.intakeUp();
            },
            intake);
    }

    public static Command down(IntakeSubsystem intake){
        return Commands.run(
            () -> {
                intake.intakeDown();
            },
            intake);
    }
}
