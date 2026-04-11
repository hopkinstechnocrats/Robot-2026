package frc.robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EmptySubsystem extends SubsystemBase{

    MatchTimer matchTimer;

    EmptySubsystem(){
        matchTimer = new MatchTimer();
    }

    @Override
    public void periodic() {
        matchTimer.update();
    }
}
