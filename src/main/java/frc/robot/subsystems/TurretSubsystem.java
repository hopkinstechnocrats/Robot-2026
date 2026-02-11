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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    NetworkTableInstance inst;
    NetworkTable table;
    TalonFX m_turretMotor;
    DoubleEntry PIDDifference;
    DoubleEntry MotorVoltage; 
    Slot0Configs m_turretConfig;
    MotorOutputConfigs m_turretOutputConfig; 
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

        m_turretSetpoint = m_turretProfile.calculate(0.020, m_turretSetpoint, m_turretGoal);
        m_request.Position = m_turretSetpoint.position;
        m_request.Velocity = m_turretSetpoint.velocity;

        m_turretOutputConfig.NeutralMode = NeutralModeValue.Brake;
        m_turretMotor.setNeutralMode(NeutralModeValue.Brake);
        m_turretMotor.getConfigurator().apply(m_turretOutputConfig);
        m_turretMotor.getConfigurator().apply(m_turretConfig);

    }

    @Override
    public void periodic(){//HERE
      	PIDDifference.set(m_intakeDeployMotor.getClosedLoopError().getValueAsDouble()); 
      	PIDFollowerDifference.set(m_intakeDeployMotorFollower.getClosedLoopError().getValueAsDouble()); 
     	//difference between desired state and real state as a double
		MotorVoltage.set(m_intakeDeployMotor.getMotorVoltage().getValueAsDouble());
        MotorFollowerVoltage.set(m_intakeDeployMotorFollower.getMotorVoltage().getValueAsDouble());
    }

    public void turret(double turretSpeed){
        m_turretMotor.set(Constants.TurretConstants.k_turretSpeedRPS);
    }

    public void turretBrake(double turretSpeed){
        m_turretMotor.set(Constants.TurretConstants.k_turretBrakeSpeedRPS);
    }
    
    
}
