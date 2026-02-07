package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase{
    TalonFX hopperMotor;
        public HopperSubsystem(){
            hopperMotor = new TalonFX(Constants.hopperMotorCANID); //Need to getCANID
            hopperMotor.setNeutralMode(NeutralModeValue.Brake);
        }
    
        public void hopper(double hopperSpeed){
            hopperMotor.set(hopperSpeed);
        }
}
