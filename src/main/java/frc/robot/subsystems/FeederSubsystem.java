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


public class FeederSubsystem extends SubsystemBase {
    TalonFX m_feederMotor;
    NetworkTableInstance inst;
    NetworkTable table;
  	DoubleEntry FeederPIDDifference; 
    DoubleEntry FeederMotorVoltage; 
    Slot0Configs m_feederConfig;
    MotorOutputConfigs m_feederOutputConfig;
    final VelocityVoltage m_feederRequest = new VelocityVoltage(0).withSlot(0);

    public FeederSubsystem(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Feeder Info");
        m_feederMotor = new TalonFX(Constants.FeederConstants.feederMotorCANID);
        m_feederMotor.setNeutralMode(NeutralModeValue.Brake);
        m_feederConfig = new Slot0Configs();
        m_feederOutputConfig = new MotorOutputConfigs();
        m_feederConfig.kP = Constants.FeederConstants.k_feederP;
        m_feederConfig.kI = Constants.FeederConstants.k_feederI;
        m_feederConfig.kD = Constants.FeederConstants.k_feederD;
		m_feederConfig.kV = Constants.FeederConstants.feederFeedForward;
        m_feederMotor.getConfigurator().apply(m_feederConfig);

        FeederMotorVoltage = table.getDoubleTopic("Feeder Motor Volated").getEntry(0);
        FeederPIDDifference = table.getDoubleTopic("Feeder PID Difference").getEntry(0);
    }

    @Override
    public void periodic(){
      	FeederPIDDifference.set(m_feederMotor.getClosedLoopError().getValueAsDouble()); 
     	//difference between desired state and real state as a double
		FeederMotorVoltage.set(m_feederMotor.getMotorVoltage().getValueAsDouble());
    }

    public void feeder(double feederSpeed){
        m_feederMotor.setControl(m_feederRequest.withVelocity(feederSpeed));
    }
}
