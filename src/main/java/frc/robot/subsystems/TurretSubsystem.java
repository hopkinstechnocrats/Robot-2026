package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.TunableNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase{
    TalonFX m_turretMotor;
    NetworkTableInstance inst;
    NetworkTable table;
    
  	DoubleEntry turretPIDDifference; 
    //DoubleEntry turretMotorVoltage;

    TunableNumber k_PInputturret; 
    TunableNumber k_IInputturret; 
    TunableNumber k_DInputturret;
    TunableNumber k_FeedForewardturret;
    
    Slot0Configs m_turretConfig;
    MotorOutputConfigs m_turretOutputConfig;
    CurrentLimitsConfigs m_currentLimits;
    final PositionVoltage m_turretRequest = new PositionVoltage(0).withSlot(0);

    public TurretSubsystem(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("turret Info");
        m_turretMotor = new TalonFX(Constants.TurretConstants.k_turretMotorCANID);
        m_turretConfig = new Slot0Configs();
        m_turretOutputConfig = new MotorOutputConfigs();
        m_currentLimits = new CurrentLimitsConfigs();
        m_turretConfig.kP = Constants.TurretConstants.k_turretP;
        m_turretConfig.kI = Constants.TurretConstants.k_turretI;
        m_turretConfig.kD = Constants.TurretConstants.k_turretD;
		m_turretConfig.kV = Constants.TurretConstants.k_turretFeedForward;
        m_turretOutputConfig.NeutralMode = NeutralModeValue.Coast;
        m_currentLimits.StatorCurrentLimit = 80;
        m_turretMotor.getConfigurator().apply(m_currentLimits);
        m_turretMotor.getConfigurator().apply(m_turretOutputConfig);
        m_turretMotor.getConfigurator().apply(m_turretConfig);
        
        //turretMotorVoltage = table.getDoubleTopic("turret Motor Volated").getEntry(0);
        turretPIDDifference = table.getDoubleTopic("turret PID Difference").getEntry(0);
        
        k_PInputturret = new TunableNumber("/Tunable Numbers/kPInput turret", Constants.TurretConstants.k_turretP);
        k_IInputturret = new TunableNumber("/Tunable Numbers/kIInput turret", Constants.TurretConstants.k_turretI);
        k_DInputturret = new TunableNumber("/Tunable Numbers/kDInput turret", Constants.TurretConstants.k_turretD);
        k_FeedForewardturret = new TunableNumber("/Tunable Numbers/FeedForeward Input turret", Constants.TurretConstants.k_turretFeedForward);
        
    }

    @Override
    public void periodic(){
        
      	turretPIDDifference.set(m_turretMotor.getClosedLoopError().getValueAsDouble()); 
     	//difference between desired state and real state as a double
		//turretMotorVoltage.set(m_turretMotor.getMotorVoltage().getValueAsDouble());
        
        if(DriverStation.isTestEnabled() && k_PInputturret.hasChanged(hashCode())){
                m_turretConfig.kP = k_PInputturret.getAsDouble();
                m_turretMotor.getConfigurator().apply(m_turretConfig);
            }

        if(DriverStation.isTestEnabled() && k_IInputturret.hasChanged(hashCode())){
                m_turretConfig.kI = k_IInputturret.getAsDouble();
                m_turretMotor.getConfigurator().apply(m_turretConfig);
            }

        if(DriverStation.isTestEnabled() && k_DInputturret.hasChanged(hashCode())){
                m_turretConfig.kD = k_DInputturret.getAsDouble();
                m_turretMotor.getConfigurator().apply(m_turretConfig);
            }

        if(DriverStation.isTestEnabled() && k_FeedForewardturret.hasChanged(hashCode())){
                m_turretConfig.kV = k_FeedForewardturret.getAsDouble();
                m_turretMotor.getConfigurator().apply(m_turretConfig);
            }
            
    }

    public void turretBrake(){
        m_turretMotor.setControl(m_turretRequest.withPosition(0));
    }
}

