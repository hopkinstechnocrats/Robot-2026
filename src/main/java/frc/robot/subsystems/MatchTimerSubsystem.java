package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MatchTimer;

public class MatchTimerSubsystem extends SubsystemBase{

    MatchTimer matchTimer;

    MatchTimerSubsystem(){
        matchTimer = new MatchTimer();
    }

    @Override
    public void periodic() {
        matchTimer.update();
    }
}