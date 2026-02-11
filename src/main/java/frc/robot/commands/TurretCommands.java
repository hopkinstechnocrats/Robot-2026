package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommands extends Command {

    public static Command turret(TurretSubsystem wheel) {
        return Commands.run(
            () -> {
                wheel.turret(Constants.TurretConstants.k_turretSpeedRPS);
            },
        wheel);
    }
    public static Command reverseTurret(TurretSubsystem wheel) {
        return Commands.run(
            () -> {
                wheel.turret(Constants.TurretConstants.k_reverseTurretSpeedRPS);
            },
        wheel);
    }
}