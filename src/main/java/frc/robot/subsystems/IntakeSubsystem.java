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



    public class IntakeSubsystem extends SubsystemBase{
        NetworkTableInstance inst;
      	NetworkTable table;
        TalonFX m_intakeMotor;
        TalonFX m_intakeDeployMotor;
        TalonFX m_intakeDeployMotorFollower;
        DoubleEntry PIDDifference; 
        DoubleEntry PIDFollowerDifference; 
		DoubleEntry MotorVoltage; 
		DoubleEntry MotorFollowerVoltage;         
        Slot0Configs m_intakeConfig;
        Slot1Configs m_intakeDeployConfig;
        MotorOutputConfigs m_intakeOutputConfig;
        MotorOutputConfigs m_intakeDeployOutputConfig;
        public final static VelocityVoltage m_intakeRequest = new VelocityVoltage(0).withSlot(0);
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        public IntakeSubsystem(){
            inst = NetworkTableInstance.getDefault();
            table = inst.getTable("Launcher Info");
            m_intakeMotor = new TalonFX(Constants.intakeMotorCANID); //Need to getCANID
            m_intakeDeployMotor = new TalonFX(Constants.intakeDeployMotorCANID); //Also needs CANID
            m_intakeDeployMotorFollower = new TalonFX(Constants.intakeDeployMotorFollowerCANID); //Also needs CANID
            Slot0Configs m_intakeConfig = new Slot0Configs();
            Slot1Configs m_intakeDeployConfig = new Slot1Configs();
            m_intakeOutputConfig = new MotorOutputConfigs();
            m_intakeConfig.kP = Constants.k_intakeP;
            m_intakeConfig.kI = Constants.k_intakeI;
            m_intakeConfig.kD = Constants.k_intakeD;
            m_intakeDeployOutputConfig = new MotorOutputConfigs();
            m_intakeDeployConfig.kP = Constants.k_intakeDeployP;
            m_intakeDeployConfig.kI = Constants.k_intakeDeployI;
            m_intakeDeployConfig.kD = Constants.k_intakeDeployD;
            m_intakeDeployConfig.kS = Constants.k_intakeDeployS;
            m_intakeDeployConfig.kV = Constants.k_intakeDeployV;

            final TrapezoidProfile m_intakeDeployProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(80, 160));
            TrapezoidProfile.State m_DeployGoal = new TrapezoidProfile.State(Constants.k_IntakePosition, 0);
            TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
            
            m_setpoint = m_intakeDeployProfile.calculate(0.020, m_setpoint, m_DeployGoal);
            m_request.Position = m_setpoint.position;
            m_request.Velocity = m_setpoint.velocity;

            m_intakeOutputConfig.NeutralMode = NeutralModeValue.Brake;
            m_intakeDeployOutputConfig.NeutralMode = NeutralModeValue.Brake;
            m_intakeMotor.setNeutralMode(NeutralModeValue.Brake);
            m_intakeDeployMotor.setNeutralMode(NeutralModeValue.Brake);
            m_intakeDeployMotor.getConfigurator().apply(m_intakeDeployOutputConfig);
            m_intakeDeployMotorFollower.setNeutralMode(NeutralModeValue.Brake);
            m_intakeDeployMotorFollower.getConfigurator().apply(m_intakeDeployOutputConfig);
            m_intakeMotor.getConfigurator().apply(m_intakeConfig);

            

        }
    @Override
    public void periodic(){
      		PIDDifference.set(m_intakeDeployMotor.getClosedLoopError().getValueAsDouble()); 
      		PIDFollowerDifference.set(m_intakeDeployMotorFollower.getClosedLoopError().getValueAsDouble()); 
     		//difference between desired state and real state as a double
			MotorVoltage.set(m_intakeDeployMotor.getMotorVoltage().getValueAsDouble());
            MotorFollowerVoltage.set(m_intakeDeployMotorFollower.getMotorVoltage().getValueAsDouble());
    }
        
        public void intake(double intakeSpeed){
            m_intakeMotor.setControl(m_intakeRequest.withVelocity(10));
        }
        public void intakeDeploy(double intakePosition){
            m_intakeDeployMotor.setControl(m_request.withPosition(m_request.Position).withVelocity(m_request.Velocity));
            m_intakeDeployMotorFollower.setControl(m_request.withPosition(-m_request.Position).withVelocity(-m_request.Velocity));

        }

}