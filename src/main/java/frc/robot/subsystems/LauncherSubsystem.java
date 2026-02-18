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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TunableNumber;


    public class LauncherSubsystem extends SubsystemBase{
      	NetworkTableInstance inst;
      	NetworkTable table;
      	DoubleEntry LauncherPIDDifference; 
		DoubleEntry LauncherMotorVoltage; 
      	TalonFX m_launcherMotor;
        TalonFX m_launcherMotorSecond;
   		Slot0Configs m_launcherConfig;
        Slot0Configs m_launcherInvertedConfig;
        MotorOutputConfigs m_launcherOutputConfig;
        MotorOutputConfigs m_launcherInvertedOutputConfig;
        TunableNumber kPInputLauncher;
        TunableNumber kIInputLauncher;
        TunableNumber kDInputLauncher;
        TunableNumber kVInputLauncher;
        final VelocityVoltage m_launcherRequest = new VelocityVoltage(0).withSlot(0);
        
		public LauncherSubsystem(){
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("Launcher Info");
            m_launcherMotor = new TalonFX(Constants.LauncherConstants.k_launcherMotorCANID); //Need to getCANID
            m_launcherMotorSecond = new TalonFX(Constants.LauncherConstants.k_launcherMotorSecondCANID); //need CANID
            m_launcherInvertedConfig = new Slot0Configs();
            m_launcherConfig = new Slot0Configs();
            m_launcherOutputConfig = new MotorOutputConfigs();
            m_launcherInvertedOutputConfig = new MotorOutputConfigs();
            m_launcherConfig.kP = Constants.LauncherConstants.k_launcherP;
            m_launcherConfig.kI = Constants.LauncherConstants.k_launcherI;
            m_launcherConfig.kD = Constants.LauncherConstants.k_launcherD;
			m_launcherConfig.kV = Constants.LauncherConstants.k_launcherFeedForward;
            m_launcherConfig.kP = Constants.LauncherConstants.k_launcherP;
            m_launcherConfig.kI = Constants.LauncherConstants.k_launcherI;
            m_launcherConfig.kD = Constants.LauncherConstants.k_launcherD;
			m_launcherConfig.kV = Constants.LauncherConstants.k_launcherFeedForward;
            m_launcherOutputConfig.NeutralMode = NeutralModeValue.Brake;
            m_launcherInvertedOutputConfig.NeutralMode = NeutralModeValue.Brake;
            m_launcherOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
            m_launcherInvertedOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

            m_launcherMotor.setNeutralMode(NeutralModeValue.Brake);
            m_launcherMotorSecond.setNeutralMode(NeutralModeValue.Brake);
            m_launcherMotor.getConfigurator().apply(m_launcherOutputConfig);
            m_launcherMotorSecond.setNeutralMode(NeutralModeValue.Brake);
            m_launcherMotorSecond.getConfigurator().apply(m_launcherOutputConfig);
            m_launcherMotor.getConfigurator().apply(m_launcherConfig);

			LauncherMotorVoltage = table.getDoubleTopic("Motor Volated").getEntry(0);
            LauncherPIDDifference = table.getDoubleTopic("PID Difference").getEntry(0);

            kPInputLauncher = new TunableNumber("/Tunable Numbers/kPInput Launcher", Constants.LauncherConstants.k_launcherP);
            kIInputLauncher = new TunableNumber("/Tunable Numbers/kIInput Launcher", Constants.LauncherConstants.k_launcherI);
            kDInputLauncher = new TunableNumber("/Tunable Numbers/kDInput Launcher", Constants.LauncherConstants.k_launcherD);
            kVInputLauncher = new TunableNumber("/Tunable Numbers/kVInput Launcher", Constants.LauncherConstants.k_launcherFeedForward);
  
        }
        
		
		@Override
    	public void periodic(){
      		LauncherPIDDifference.set(m_launcherMotor.getClosedLoopError().getValueAsDouble()); 
     		//difference between desired state and real state as a double
			LauncherMotorVoltage.set(m_launcherMotor.getMotorVoltage().getValueAsDouble());
            if(DriverStation.isTestEnabled() && kPInputLauncher.hasChanged(hashCode())){
                m_launcherConfig.kP = kPInputLauncher.getAsDouble();
                m_launcherMotor.getConfigurator().apply(m_launcherConfig);
            }

            if(DriverStation.isTestEnabled() && kIInputLauncher.hasChanged(hashCode())){
                m_launcherConfig.kI = kIInputLauncher.getAsDouble();
                m_launcherMotor.getConfigurator().apply(m_launcherConfig);
            }

            if(DriverStation.isTestEnabled() && kDInputLauncher.hasChanged(hashCode())){
                m_launcherConfig.kD = kDInputLauncher.getAsDouble();
                m_launcherMotor.getConfigurator().apply(m_launcherConfig);
            }

            if(DriverStation.isTestEnabled() && kVInputLauncher.hasChanged(hashCode())){
                m_launcherConfig.kV = kVInputLauncher.getAsDouble();
                m_launcherMotor.getConfigurator().apply(m_launcherConfig);
            }
    	}
        
        public void launcher(double launcherSpeed){
        	m_launcherMotor.setControl(m_launcherRequest.withVelocity(launcherSpeed));
        }
    }

