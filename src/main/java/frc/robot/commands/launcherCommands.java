package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.launcherSubsystem;

public class launcherCommands extends Command {
       
    public static Command launcher(launcherSubsystem wheel, double launcherSpeed) {
        return Commands.run(
            () -> {
                wheel.launcher(launcherSpeed);
            },
        wheel);
    }
    public static Command reverseLauncher (launcherSubsystem wheel) {
        return Commands.run(
            () -> {
                wheel.launcher(Constants.LauncherConstants.k_reverseLauncherSpeedRPS);
            },
        wheel);
    }
}
