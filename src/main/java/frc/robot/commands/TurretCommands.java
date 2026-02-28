package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommands extends Command {

    public static Command turret(TurretSubsystem turret) {
        return Commands.run(
            () -> {
                turret.turret(Constants.TurretConstants.k_turretSpeedRPS);
            },
        turret);
    }
    public static Command reverseTurret(TurretSubsystem reverseTurret) {
        return Commands.run(
            () -> {
                reverseTurret.turret(Constants.TurretConstants.k_reverseTurretSpeedRPS);
            },
        reverseTurret);
    }
    public static Command TurretP1(TurretSubsystem TurretP1) {
        return Commands.run(
            () -> {
                TurretP1.turretP1();
            },
        TurretP1);
    }
    public static Command TurretP2(TurretSubsystem TurretP2) {
        return Commands.run(
            () -> {
                TurretP2.turretP2();
            },
        TurretP2);
    }
    public static Command TurretP3(TurretSubsystem TurretP3) {
        return Commands.run(
            () -> {
                TurretP3.turretP3();
            },
        TurretP3);
    }
}