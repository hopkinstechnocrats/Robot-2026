package frc.robot.subsystems;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TunableNumber;

public class HopperSubsystem extends SubsystemBase{

 	NetworkTableInstance inst;
  	NetworkTable table;
  	DoubleEntry HopperPIDDifference; 
	DoubleEntry HopperMotorVoltage; 
  	TalonFX m_hopperMotor;
    TalonFXConfiguration m_talonConfig;
    
    TunableNumber kPInputHopper;
    TunableNumber kIInputHopper;
    TunableNumber kDInputHopper;
    final VelocityVoltage m_hopperRequest = new VelocityVoltage(0).withSlot(0);

    TalonFX hopperMotor;

        public HopperSubsystem(){
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("Hopper Info");

            kPInputHopper = new TunableNumber("/Tunable Numbers/kPInput Hopper", Constants.HopperConstants.k_hopperP);
            kIInputHopper = new TunableNumber("/Tunable Numbers/kIInput Hopper", Constants.HopperConstants.k_hopperI);
            kDInputHopper = new TunableNumber("/Tunable Numbers/kDInput Hopper", Constants.HopperConstants.k_hopperD);

            m_hopperMotor = new TalonFX(Constants.HopperConstants.k_hopperMotorCANID); //Need to getCANID
            m_talonConfig = new TalonFXConfiguration();
            
            m_talonConfig.Slot0.kP = Constants.HopperConstants.k_hopperP;
            m_talonConfig.Slot0.kI = Constants.HopperConstants.k_hopperI;
            m_talonConfig.Slot0.kD = Constants.HopperConstants.k_hopperD;
			m_talonConfig.Slot0.kV = Constants.HopperConstants.k_feedForward;
            m_talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;
            m_talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_hopperMotor.getConfigurator().apply(m_talonConfig);

            HopperMotorVoltage = table.getDoubleTopic("Hopper Motor Volated").getEntry(0);
            HopperPIDDifference = table.getDoubleTopic("Hopper PID Difference").getEntry(0);
        }
    
		@Override
    	public void periodic(){
      		HopperPIDDifference.set(m_hopperMotor.getClosedLoopError().getValueAsDouble()); 
     		//difference between desired state and real state as a double
			HopperMotorVoltage.set(m_hopperMotor.getMotorVoltage().getValueAsDouble());

            if(DriverStation.isTestEnabled() && kPInputHopper.hasChanged(hashCode())){
                m_talonConfig.Slot0.kP = kPInputHopper.getAsDouble();
                m_hopperMotor.getConfigurator().apply(m_talonConfig);
            }

            if(DriverStation.isTestEnabled() && kIInputHopper.hasChanged(hashCode())){
                m_talonConfig.Slot0.kI = kIInputHopper.getAsDouble();
                m_hopperMotor.getConfigurator().apply(m_talonConfig);
            }

            if(DriverStation.isTestEnabled() && kDInputHopper.hasChanged(hashCode())){
                m_talonConfig.Slot0.kD = kDInputHopper.getAsDouble();
                m_hopperMotor.getConfigurator().apply(m_talonConfig);            
            }
    	}

        public void hopper(double hopperSpeed){
        	m_hopperMotor.setControl(m_hopperRequest.withVelocity(hopperSpeed));
        }

        public void hopperBrake(){
        	m_hopperMotor.setControl(m_hopperRequest.withVelocity(Constants.HopperConstants.k_hopperBrakeSpeedRPS));
        }
}
