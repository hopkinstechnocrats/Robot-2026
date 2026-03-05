package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

public class HopperCommands extends Command {
       
    public static Command hopper(HopperSubsystem hoptest) {
        return Commands.run(
            () -> {
                hoptest.hopper(Constants.HopperConstants.k_hopperSpeedRPS);
            },
        hoptest);
    }
    public static Command reverseHopper(HopperSubsystem hoptest) {
        return Commands.run(
            () -> {
                hoptest.hopper(Constants.HopperConstants.k_reverseHopperSpeedRPS);
            },
        hoptest);
    }
}
