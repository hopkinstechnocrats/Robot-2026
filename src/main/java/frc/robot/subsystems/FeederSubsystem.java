package frc.robot.subsystems;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.TunableNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
    TalonFX m_feederMotor;
    NetworkTableInstance inst;
    NetworkTable table;
  	DoubleEntry FeederPIDDifference; 
    DoubleEntry FeederMotorVoltage;
    TunableNumber k_PInputFeeder; 
    TunableNumber k_IInputFeeder; 
    TunableNumber k_DInputFeeder;
    TunableNumber k_FeedForewardFeeder;
    Slot0Configs m_feederConfig;
    MotorOutputConfigs m_feederOutputConfig;
    final VelocityVoltage m_feederRequest = new VelocityVoltage(0).withSlot(0);

    public FeederSubsystem(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Feeder Info");
        m_feederMotor = new TalonFX(Constants.FeederConstants.k_feederMotorCANID);
        m_feederMotor.setNeutralMode(NeutralModeValue.Brake);
        m_feederConfig = new Slot0Configs();
        m_feederOutputConfig = new MotorOutputConfigs();
        m_feederConfig.kP = Constants.FeederConstants.k_feederP;
        m_feederConfig.kI = Constants.FeederConstants.k_feederI;
        m_feederConfig.kD = Constants.FeederConstants.k_feederD;
		m_feederConfig.kV = Constants.FeederConstants.k_feederFeedForward;
        m_feederMotor.getConfigurator().apply(m_feederConfig);

        FeederMotorVoltage = table.getDoubleTopic("Feeder Motor Volated").getEntry(0);
        FeederPIDDifference = table.getDoubleTopic("Feeder PID Difference").getEntry(0);
        k_PInputFeeder = new TunableNumber("/Tunable Numbers/kPInput Feeder", Constants.FeederConstants.k_feederP);
        k_IInputFeeder = new TunableNumber("/Tunable Numbers/kIInput Feeder", Constants.FeederConstants.k_feederI);
        k_DInputFeeder = new TunableNumber("/Tunable Numbers/kDInput Feeder", Constants.FeederConstants.k_feederD);
        k_FeedForewardFeeder = new TunableNumber("/Tunable Numbers/FeedForeward Input Feeder", Constants.FeederConstants.k_feederFeedForward);
    }

    @Override
    public void periodic(){
      	FeederPIDDifference.set(m_feederMotor.getClosedLoopError().getValueAsDouble()); 
     	//difference between desired state and real state as a double
		FeederMotorVoltage.set(m_feederMotor.getMotorVoltage().getValueAsDouble());

        if(DriverStation.isTestEnabled() && k_PInputFeeder.hasChanged(hashCode())){
                m_feederConfig.kP = k_PInputFeeder.getAsDouble();
                m_feederMotor.getConfigurator().apply(m_feederConfig);
            }

        if(DriverStation.isTestEnabled() && k_IInputFeeder.hasChanged(hashCode())){
                m_feederConfig.kI = k_IInputFeeder.getAsDouble();
                m_feederMotor.getConfigurator().apply(m_feederConfig);
            }

        if(DriverStation.isTestEnabled() && k_DInputFeeder.hasChanged(hashCode())){
                m_feederConfig.kD = k_DInputFeeder.getAsDouble();
                m_feederMotor.getConfigurator().apply(m_feederConfig);
            }

        if(DriverStation.isTestEnabled() && k_FeedForewardFeeder.hasChanged(hashCode())){
                m_feederConfig.kV = k_FeedForewardFeeder.getAsDouble();
                m_feederMotor.getConfigurator().apply(m_feederConfig);
            }
    }

    public void feeder(double feederSpeed){
        m_feederMotor.setControl(m_feederRequest.withVelocity(feederSpeed));
    }

    public void feederBrake(){
        m_feederMotor.setControl(m_feederRequest.withVelocity(Constants.FeederConstants.k_feederBreakSpeedRPS));
    }
}
