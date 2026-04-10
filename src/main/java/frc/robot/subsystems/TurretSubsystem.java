package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TunableNumber;

    public class TurretSubsystem extends SubsystemBase{

        NetworkTableInstance inst;
        NetworkTable table;
        
        DoubleEntry TurretPIDDifference; 

        TalonFX m_turretMotor;
        Slot0Configs m_turretConfig;
        MotorOutputConfigs m_turretOutputConfig;
        CurrentLimitsConfigs m_turretCurrentLimits;

        TunableNumber kPInputTurret;
        TunableNumber kIInputTurret;
        TunableNumber kDInputTurret;
        TunableNumber kVInputTurret;

        final VelocityVoltage m_turretRequest = new VelocityVoltage(0).withSlot(0);

        public TurretSubsystem(){

            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("Turret Info");

            m_turretMotor = new TalonFX(Constants.TurretConstants.k_turretMotorCANID);
            m_turretConfig = new Slot0Configs();
            m_turretCurrentLimits = new CurrentLimitsConfigs();
            m_turretOutputConfig = new MotorOutputConfigs();
            m_turretConfig.kP = Constants.TurretConstants.k_turretP;
            m_turretConfig.kI = Constants.TurretConstants.k_turretI;
            m_turretConfig.kD = Constants.TurretConstants.k_turretD;
			m_turretConfig.kV = Constants.TurretConstants.k_turretFeedForward;
            m_turretOutputConfig.NeutralMode = NeutralModeValue.Brake;
            m_turretCurrentLimits.StatorCurrentLimit = 80;

            m_turretMotor.getConfigurator().apply(m_turretCurrentLimits);
            m_turretMotor.getConfigurator().apply(m_turretOutputConfig);
            m_turretMotor.getConfigurator().apply(m_turretConfig);

            TurretPIDDifference = table.getDoubleTopic("PID Difference").getEntry(0);

            kPInputTurret = new TunableNumber("/Tunable Numbers/kPInput Launcher", Constants.LauncherConstants.k_launcherP);
            kIInputTurret = new TunableNumber("/Tunable Numbers/kIInput Launcher", Constants.LauncherConstants.k_launcherI);
            kDInputTurret = new TunableNumber("/Tunable Numbers/kDInput Launcher", Constants.LauncherConstants.k_launcherD);
            kVInputTurret = new TunableNumber("/Tunable Numbers/kVInput Launcher", Constants.LauncherConstants.k_launcherFeedForward);

        
        }  
        public void turretSpin(double turretSpeed){
        	m_turretMotor.setControl(m_turretRequest.withVelocity(turretSpeed));
        }
        public void turretBrake(double turretSpeed){
        	m_turretMotor.setControl(m_turretRequest.withVelocity(turretSpeed));
        }
}

