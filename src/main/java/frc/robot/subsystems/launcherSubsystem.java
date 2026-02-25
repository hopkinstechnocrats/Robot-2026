package frc.robot.subsystems;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TunableNumber;

public class launcherSubsystem extends SubsystemBase{

 	NetworkTableInstance inst;
  	NetworkTable table;
  	DoubleEntry launcherPIDDifference; 
    DoubleEntry launcherPIDDifferenceSecond; 
	DoubleEntry launcherMotorVoltage; 
	DoubleEntry launcherMotorVoltageSecond; 
  	TalonFX m_launcherMotor;
  	TalonFX m_launcherMotorSecond;
	Slot0Configs m_launcherConfig;
	Slot1Configs m_launcherConfigSecond;
    MotorOutputConfigs m_launcherOutputConfig;
    MotorOutputConfigs m_launcherOutputConfigSecond;
    TunableNumber kPInputLauncher;
    TunableNumber kIInputLauncher;
    TunableNumber kDInputLauncher;
    final VelocityVoltage m_launcherRequest = new VelocityVoltage(0).withSlot(0);

    TalonFX launcherMotor;
    TalonFX launcherMotorSecond;

        public launcherSubsystem(){
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("launcher Info");

            kPInputLauncher = new TunableNumber("/Tunable Numbers/kPInput launcher", Constants.LauncherConstants.k_launcherP);
            kIInputLauncher = new TunableNumber("/Tunable Numbers/kIInput launcher", Constants.LauncherConstants.k_launcherI);
            kDInputLauncher = new TunableNumber("/Tunable Numbers/kDInput launcher", Constants.LauncherConstants.k_launcherD);

            m_launcherMotor = new TalonFX(Constants.LauncherConstants.k_launcherMotorCANID); //Need to getCANID
            m_launcherMotorSecond = new TalonFX(Constants.LauncherConstants.k_launcherMotorSecondCANID); //Need to getCANID
            m_launcherConfig = new Slot0Configs();
            m_launcherConfigSecond = new Slot1Configs();
            m_launcherOutputConfig = new MotorOutputConfigs();
            m_launcherOutputConfigSecond = new MotorOutputConfigs();
            m_launcherConfig.kP = Constants.LauncherConstants.k_launcherP;
            m_launcherConfig.kI = Constants.LauncherConstants.k_launcherI;
            m_launcherConfig.kD = Constants.LauncherConstants.k_launcherD;
			m_launcherConfig.kV = Constants.LauncherConstants.k_feedForward;
            m_launcherConfigSecond.kP = Constants.LauncherConstants.k_launcherP;
            m_launcherConfigSecond.kI = Constants.LauncherConstants.k_launcherI;
            m_launcherConfigSecond.kD = Constants.LauncherConstants.k_launcherD;
			m_launcherConfigSecond.kV = Constants.LauncherConstants.k_feedForward;
            m_launcherOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
            m_launcherOutputConfigSecond.Inverted = InvertedValue.CounterClockwise_Positive;
            m_launcherMotor.getConfigurator().apply(m_launcherConfig);
            m_launcherMotorSecond.getConfigurator().apply(m_launcherConfigSecond);

            launcherMotorVoltage = table.getDoubleTopic("launcher Motor Volated").getEntry(0);
            launcherMotorVoltageSecond = table.getDoubleTopic("launcher Motor Volated").getEntry(0);
            launcherPIDDifference = table.getDoubleTopic("launcher PID Difference").getEntry(0);
            launcherPIDDifferenceSecond = table.getDoubleTopic("launcher PID Difference").getEntry(0);
        }
    
		@Override
    	public void periodic(){
      		launcherPIDDifference.set(m_launcherMotor.getClosedLoopError().getValueAsDouble()); 
     		launcherPIDDifferenceSecond.set(m_launcherMotorSecond.getClosedLoopError().getValueAsDouble()); 
            //difference between desired state and real state as a double
			launcherMotorVoltage.set(m_launcherMotor.getMotorVoltage().getValueAsDouble());
            launcherMotorVoltageSecond.set(m_launcherMotorSecond.getMotorVoltage().getValueAsDouble());

            if(DriverStation.isTestEnabled() && kPInputLauncher.hasChanged(hashCode())){
                m_launcherConfig.kP = kPInputLauncher.getAsDouble();
                m_launcherMotor.getConfigurator().apply(m_launcherConfig);
                m_launcherConfigSecond.kP = kPInputLauncher.getAsDouble();
                m_launcherMotorSecond.getConfigurator().apply(m_launcherConfigSecond);
            }

            if(DriverStation.isTestEnabled() && kIInputLauncher.hasChanged(hashCode())){
                m_launcherConfig.kI = kIInputLauncher.getAsDouble();
                m_launcherMotor.getConfigurator().apply(m_launcherConfig);
                m_launcherConfigSecond.kI = kIInputLauncher.getAsDouble();
                m_launcherMotorSecond.getConfigurator().apply(m_launcherConfigSecond);
            }

            if(DriverStation.isTestEnabled() && kDInputLauncher.hasChanged(hashCode())){
                m_launcherConfig.kD = kDInputLauncher.getAsDouble();
                m_launcherMotor.getConfigurator().apply(m_launcherConfig);
                m_launcherConfigSecond.kD = kDInputLauncher.getAsDouble();
                m_launcherMotorSecond.getConfigurator().apply(m_launcherConfigSecond);
            }
    	}

        public void launcher(double launcherSpeed){
        	m_launcherMotor.setControl(m_launcherRequest.withVelocity(launcherSpeed));
        	m_launcherMotorSecond.setControl(m_launcherRequest.withVelocity(launcherSpeed));
        }

        public void launcherBrake(){
        	m_launcherMotor.setControl(m_launcherRequest.withVelocity(Constants.LauncherConstants.k_launcherBrakeSpeedRPS));
        	m_launcherMotorSecond.setControl(m_launcherRequest.withVelocity(Constants.LauncherConstants.k_launcherBrakeSpeedRPS));
        }
}
