package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
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
        TunableNumber tunableTurretPositionCenter;
        TunableNumber tunableTurretPositionRight;
        TunableNumber tunableTurretPositionLeft;

        final PositionDutyCycle m_turretRequest = new PositionDutyCycle(0).withSlot(0);

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

            kPInputTurret = new TunableNumber("/Tunable Numbers/kPInput Launcher", Constants.TurretConstants.k_turretP);
            kIInputTurret = new TunableNumber("/Tunable Numbers/kIInput Launcher", Constants.TurretConstants.k_turretI);
            kDInputTurret = new TunableNumber("/Tunable Numbers/kDInput Launcher", Constants.TurretConstants.k_turretD);
            kVInputTurret = new TunableNumber("/Tunable Numbers/kVInput Launcher", Constants.TurretConstants.k_turretFeedForward);
            tunableTurretPositionCenter = new TunableNumber("/Tunable Numbers/Turret Position Center", Constants.TurretConstants.k_turretCenterPosition);
            tunableTurretPositionRight = new TunableNumber("/Tunable Numbers/Turret Position Right", Constants.TurretConstants.k_turretRightPosition);
            tunableTurretPositionLeft = new TunableNumber("/Tunable Numbers/Turret Position Left", Constants.TurretConstants.k_turretLeftPosition);
        }  



public void turretPositionCenter(double position){
            m_turretMotor.setControl(m_turretRequest.withPosition(Constants.TurretConstants.k_turretCenterPosition));
        }

public void turretPositionRight(double position){
            m_turretMotor.setControl(m_turretRequest.withPosition(Constants.TurretConstants.k_turretRightPosition));
        }

public void turretPositionLeft(double position){
            m_turretMotor.setControl(m_turretRequest.withPosition(Constants.TurretConstants.k_turretLeftPosition));
        }
}

