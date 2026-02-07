package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.Constants;

public class LauncherCommands extends Command {
    
    public static Command launcher(LauncherSubsystem wheel) {
        return Commands.run(
            () -> {
                wheel.launcher(Constants.LauncherConstants.k_launchSpeedRPS);
            },
        wheel);
    }
     public static Command launcherBreak(LauncherSubsystem wheel){
        return Commands.run(
            () -> {
                wheel.launcherBrake(Constants.LauncherConstants.k_launcherBrakeSpeedRPS);
            },
            wheel);
    }
}