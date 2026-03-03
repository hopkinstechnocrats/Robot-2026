package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommands extends Command {
       
    public static Command intake(IntakeSubsystem intake) {
        return Commands.run(
            () -> {
                intake.intake(Constants.IntakeConstants.k_intakeSpeedRPS);
            },
        intake);
    }

    public static Command outtake(IntakeSubsystem outtake) {
        return Commands.run(
            () -> {
                outtake.intake(Constants.IntakeConstants.k_reverseIntakeSpeedRPS);
            },
        outtake);
    }

    public static Command deploy(IntakeSubsystem deploy) {
        return Commands.run(
            () -> {
                deploy.intakeDeploy();
            },
        deploy);
    }

    public static Command deployBob(IntakeSubsystem deployBob) {
        return Commands.run(
            () -> {
                deployBob.intakeBob();
            },
        deployBob);
    }

    public static Command undeploy(IntakeSubsystem undeploy) {
        return Commands.run(
            () -> {
                undeploy.intakeUndeploy();
            },
        undeploy);
    }
}