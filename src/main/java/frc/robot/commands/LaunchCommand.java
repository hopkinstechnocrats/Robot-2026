package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class LaunchCommand extends Command {

    private LauncherSubsystem m_launch;
    private FeederSubsystem m_feeder;
    private HopperSubsystem m_hopper;

    public LaunchCommand(LauncherSubsystem launchSubsystem, FeederSubsystem feederSubsystem, HopperSubsystem hopperSubsystem) {
        m_launch = launchSubsystem;
        m_feeder = feederSubsystem;
        m_hopper = hopperSubsystem;

        addRequirements(launchSubsystem, feederSubsystem, hopperSubsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        LauncherCommands.launcher(m_launch).withTimeout(2)
        .andThen(FeederCommands.feeder(m_feeder).alongWith(HopperCommands.hopper(m_hopper)));
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }

}
