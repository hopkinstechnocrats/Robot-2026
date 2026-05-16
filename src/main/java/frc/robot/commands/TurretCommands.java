package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommands extends Command {
    public static Command turretBrake(TurretSubsystem turret) {
        return Commands.run(
            () -> {
                turret.turretBrake();
            },
        turret);
        
    }
}
