package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class AutonomusCommands {
    static FeederSubsystem m_FeederSubsystem;
    static FeederCommands m_FeederCommands;
    static HopperSubsystem m_HopperSubsystem;
    static HopperCommands m_HopperCommands;
    static IntakeSubsystem m_IntakeSubsystem;
    static IntakeCommands m_IntakeCommands;
    static LauncherSubsystem m_launcherSubsystem;
    static LauncherCommands m_LauncherCommands;

    AutonomusCommands(
        FeederSubsystem feederSubsystem,
        FeederCommands feederCommands,
        HopperSubsystem hopperSubsystem,
        HopperCommands hopperCommands,
        IntakeSubsystem intakeSubsystem,
        IntakeCommands intakeCommands,
        LauncherSubsystem launcherSubsystem,
        LauncherCommands launcherCommands
        ){
        m_FeederSubsystem = feederSubsystem;
        m_FeederCommands = feederCommands;
        m_HopperSubsystem = hopperSubsystem;
        m_HopperCommands = hopperCommands;
        m_IntakeSubsystem = intakeSubsystem;
        m_IntakeCommands = intakeCommands;
        m_launcherSubsystem = launcherSubsystem;
        m_LauncherCommands = launcherCommands;
    }

    Command runTimedCommand( Command inputCommand, double duration){
        return Commands.parallel(inputCommand,Commands.waitSeconds(duration));
    }

    public static Command runTimedLauncher(double durationSeconds){
        //Hyper-commented example
        return Commands.sequence( //Just runs a groups of commands in parallel

            /*The parallel just runs two commands at the same time and waits until they both finish
             * the first command starts the launcher
             * and the second command (Commands.waitSeconds) waits for the required amount of time*/
            Commands.parallel(m_LauncherCommands.launcher(m_launcherSubsystem),Commands.waitSeconds(durationSeconds)),
            
            //brake after the launcher has been running for the required amount of time.
            m_LauncherCommands.launcherBreak(m_launcherSubsystem)
        );
    }

    public static Command runTimedIntake(double durationSeconds){
        return Commands.sequence(
            Commands.parallel(m_IntakeCommands.intake(m_IntakeSubsystem),Commands.waitSeconds(durationSeconds)),
            Commands.run(()->{m_IntakeSubsystem.intakeBrake();},m_IntakeSubsystem)//brake at the end.
        );
    }

    public static Command runTimedHopper(double durationSeconds){
        return Commands.sequence(
            Commands.parallel(m_HopperCommands.hopper(m_HopperSubsystem),Commands.waitSeconds(durationSeconds)),
            m_HopperCommands.brake(m_HopperSubsystem)
        );
    }

    public static Command runTimedFeeder(double durationSeconds){
        return Commands.sequence(
            Commands.parallel(m_FeederCommands.feeder(m_FeederSubsystem), Commands.waitSeconds(durationSeconds)),
            m_FeederCommands.brakeFeeder(m_FeederSubsystem)
        );
    }

}
