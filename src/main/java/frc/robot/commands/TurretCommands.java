package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.Constants;

public class TurretCommands extends Command {

    public static Command turretSpin(LauncherSubsystem wheel) {
            return Commands.run(
                () -> {
                    wheel.launcher(Constants.TurretConstants.k_turretSpeedRPS);
                },
            wheel);
    }
    public static Command turretBrake(LauncherSubsystem wheel){
        return Commands.run(
            () -> {
                wheel.launcher(Constants.TurretConstants.k_turretBrakeSpeedRPS);
            },
            wheel);
    }

    public static Command turretReverse(LauncherSubsystem launcher){
        return Commands.run(
            () -> {
                launcher.launcher(-Constants.TurretConstants.k_turretSpeedRPS);
            },
            launcher);
    }
        
}
