package frc.robot.subsystems;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase{

 	NetworkTableInstance inst;
  	NetworkTable table;
  	DoubleEntry PIDDifference; 
	DoubleEntry MotorVoltage; 
  	TalonFX m_hopperMotor;
	Slot0Configs m_hopperConfig;
    MotorOutputConfigs m_hopperOutputConfig;
    final VelocityVoltage m_hopperRequest = new VelocityVoltage(0).withSlot(0);

    TalonFX hopperMotor;
        public HopperSubsystem(){

            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("Hopper Info");
            m_hopperMotor = new TalonFX(Constants.HopperConstants.k_hopperMotorCANID); //Need to getCANID
            m_hopperConfig = new Slot0Configs();
            m_hopperOutputConfig = new MotorOutputConfigs();
            m_hopperConfig.kP = Constants.HopperConstants.k_hopperP;
            m_hopperConfig.kI = Constants.HopperConstants.k_hopperI;
            m_hopperConfig.kD = Constants.HopperConstants.k_hopperD;
			m_hopperConfig.kV = Constants.HopperConstants.k_feedForward;
            m_hopperMotor.getConfigurator().apply(m_hopperConfig);

            MotorVoltage = table.getDoubleTopic("Motor Volated").getEntry(0);
            PIDDifference = table.getDoubleTopic("PID Difference").getEntry(0);

            hopperMotor.setNeutralMode(NeutralModeValue.Brake);
        }
    
		@Override
    	public void periodic(){
      		PIDDifference.set(m_hopperMotor.getClosedLoopError().getValueAsDouble()); 
     		//difference between desired state and real state as a double
			MotorVoltage.set(m_hopperMotor.getMotorVoltage().getValueAsDouble());
    	}

        public void hopper(double hopperSpeed){
        	m_hopperMotor.setControl(m_hopperRequest.withVelocity(Constants.HopperConstants.k_hopperSpeed));
        }
}
