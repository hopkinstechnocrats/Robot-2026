package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommands extends Command {

    public static Command feeder(FeederSubsystem wheel) {
        return Commands.run(
            () -> {
                wheel.feeder(Constants.FeederConstants.feederSpeedRPS);
            },
        wheel);
        
    }
    public static Command unfeeder(FeederSubsystem wheel) {
        return Commands.run(
            () -> {
                wheel.feeder(Constants.FeederConstants.reverseFeederSpeedRPS);
            },
        wheel);
        
    }
}