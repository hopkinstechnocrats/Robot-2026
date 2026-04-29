package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TunableNumber;

public class PWMSubsystem extends SubsystemBase{

    PWMTalonFX m_PWMintakeMotor;
    PWMTalonFX m_PWMintakeFollowerMotor;

    public PWMSubsystem(){
        m_PWMintakeMotor = new PWMTalonFX(Constants.IntakeConstants.k_intakeMotorCANID); 
        m_PWMintakeFollowerMotor = new PWMTalonFX(Constants.IntakeConstants.k_intakeFollowerCANID);
    }

    public void pwmIntake(double intakeSpeed){ 
    	m_PWMintakeMotor.set(0.5);
        m_PWMintakeFollowerMotor.set(-0.5); // this speed value should always be the speed of the above times -1
    }
}
