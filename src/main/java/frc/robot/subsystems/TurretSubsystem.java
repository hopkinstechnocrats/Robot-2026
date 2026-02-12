package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TunableNumber;

public class TurretSubsystem extends SubsystemBase {
    NetworkTableInstance inst;
    NetworkTable table;
    TalonFX m_turretMotor;
    DoubleEntry TurretPIDDifference;
    DoubleEntry TurretMotorVoltage; 
    Slot0Configs m_turretConfig;
    MotorOutputConfigs m_turretOutputConfig; 
    TunableNumber kPInputTurret;
    TunableNumber kIInputTurret;
    TunableNumber kDInputTurret;
    TunableNumber kSInputTurret;
    TunableNumber kVInputTurret;
    public final static VelocityVoltage m_intakeRequest = new VelocityVoltage(0).withSlot(0);
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    
    public TurretSubsystem(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Turret Info");
        m_turretMotor = new TalonFX(Constants.TurretConstants.k_turretMotorCANID);
        m_turretMotor.setNeutralMode(NeutralModeValue.Brake);
        Slot0Configs m_turretConfig = new Slot0Configs();
        m_turretOutputConfig = new MotorOutputConfigs();
        m_turretConfig.kP = Constants.TurretConstants.k_turretP;
        m_turretConfig.kI = Constants.TurretConstants.k_turretI;
        m_turretConfig.kD = Constants.TurretConstants.k_turretD;
        m_turretConfig.kS = Constants.TurretConstants.k_turretS;
        m_turretConfig.kV = Constants.TurretConstants.k_turretV;

        final TrapezoidProfile m_turretProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(80, 160));
        TrapezoidProfile.State m_turretGoal = new TrapezoidProfile.State(Constants.TurretConstants.k_turretPosition, 0);
        TrapezoidProfile.State m_turretSetpoint = new TrapezoidProfile.State();

        kPInputTurret = new TunableNumber("/Tunable Numbers/kPInput Turret", Constants.TurretConstants.k_turretP);
        kIInputTurret = new TunableNumber("/Tunable Numbers/kIInput Turret", Constants.TurretConstants.k_turretI);
        kDInputTurret = new TunableNumber("/Tunable Numbers/kDInput Turret", Constants.TurretConstants.k_turretD);
        kSInputTurret = new TunableNumber("/Tunable Numbers/kSInput Turret", Constants.TurretConstants.k_turretS);
        kVInputTurret = new TunableNumber("/Tunable Numbers/kVInput Turret", Constants.TurretConstants.k_turretV);

        m_turretSetpoint = m_turretProfile.calculate(0.020, m_turretSetpoint, m_turretGoal);
        m_request.Position = m_turretSetpoint.position;
        m_request.Velocity = m_turretSetpoint.velocity;

        m_turretOutputConfig.NeutralMode = NeutralModeValue.Brake;
        m_turretMotor.setNeutralMode(NeutralModeValue.Brake);
        m_turretMotor.getConfigurator().apply(m_turretOutputConfig);
    }

    @Override
    public void periodic(){
      	TurretPIDDifference.set(m_turretMotor.getClosedLoopError().getValueAsDouble());  
     	//difference between desired state and real state as a double
		TurretMotorVoltage.set(m_turretMotor.getMotorVoltage().getValueAsDouble());

        if(DriverStation.isTestEnabled() && kPInputTurret.hasChanged(hashCode())){
            m_turretConfig.kP = kPInputTurret.getAsDouble();
            m_turretMotor.getConfigurator().apply(m_turretConfig);
        }

        if(DriverStation.isTestEnabled() && kIInputTurret.hasChanged(hashCode())){
            m_turretConfig.kI = kIInputTurret.getAsDouble();
            m_turretMotor.getConfigurator().apply(m_turretConfig);
        }

        if(DriverStation.isTestEnabled() && kDInputTurret.hasChanged(hashCode())){
            m_turretConfig.kD = kDInputTurret.getAsDouble();
            m_turretMotor.getConfigurator().apply(m_turretConfig);
        }

        if(DriverStation.isTestEnabled() && kSInputTurret.hasChanged(hashCode())){
            m_turretConfig.kS = kSInputTurret.getAsDouble();
            m_turretMotor.getConfigurator().apply(m_turretConfig);
        }

        if(DriverStation.isTestEnabled() && kVInputTurret.hasChanged(hashCode())){
            m_turretConfig.kV = kVInputTurret.getAsDouble();
            m_turretMotor.getConfigurator().apply(m_turretConfig);
        }
    }

    public void turret(double turretSpeed){
        m_turretMotor.setControl(m_request.withPosition(m_request.Position).withVelocity(m_request.Velocity));
    }

    public void turretBrake(double turretSpeed){
        m_turretMotor.set(Constants.TurretConstants.k_turretBrakeSpeedRPS);
    }
    
    
}
