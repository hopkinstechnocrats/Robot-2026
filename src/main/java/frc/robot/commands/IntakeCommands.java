package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;


public class IntakeCommands {
    public static Command intake(IntakeSubsystem m_intakeSubsystem) {
        return Commands.run(
            () -> {
                m_intakeSubsystem.intake(Constants.k_intakeSpeedRPS);
            },
        m_intakeSubsystem);
    }
    public static Command outtake(IntakeSubsystem m_IntakeSubsystem){
        return Commands.run(
            () -> {
            m_IntakeSubsystem.intake(-Constants.k_intakeSpeedRPS);
        }
    );
    }
    /*public static Command deploy(IntakeSubsystem m_IntakeSubsystem){
        return Commands.run(
            () -> {
            m_IntakeSubsystem.intakeDeploy(Constants.intakeSetpoint);
            }
            );
    }*/
}
