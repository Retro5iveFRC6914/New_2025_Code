// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// falcsons instead of neos so change everything 
package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import javax.swing.text.Position;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;

/*import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;*/

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
  /** Creates a new phoenix. */  
  //Creates TalonFX objects for motors 
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    //Creates CANcoder object for the CANcoder encoder
    private final CANcoder encoder;
    //Creates MotorOutputConfigs object, and resets motor output configs
    private final MotorOutputConfigs m_MotorOutputConfigs = new MotorOutputConfigs();
    //Creates CurrentLimitsConfigs object, and resets current limits configs
    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();
   // private final DutyCycleOut m_output = new DutyCycleOut(0);
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public Elevator(int leftID, int rightID) {
//declares the motors as motors with the ID's from the arguments
    leftMotor = new TalonFX(leftID);
    rightMotor = new TalonFX(rightID);
    TalonFXConfiguration TalonFXConfigs = new TalonFXConfiguration();
    TalonFXConfigurator LeftTalonFXConfigurator = leftMotor.getConfigurator();
    TalonFXConfigurator RightTalonFXConfigurator = rightMotor.getConfigurator();
    // sets to factory default
    LeftTalonFXConfigurator.apply(new TalonFXConfiguration());
    RightTalonFXConfigurator.apply(new TalonFXConfiguration());
    //declares new cancoder       
    encoder = new CANcoder(21);
    // creates current limit for motors
    m_currentLimits.SupplyCurrentLimit = 140;
    TalonFXConfigs.CurrentLimits = m_currentLimits;
    //set current limit for left motor
    LeftTalonFXConfigurator.apply(m_currentLimits); 
    //set current limit for right motor
    RightTalonFXConfigurator.apply(m_currentLimits); 
    //inverts one motor to create rotation
    TalonFXConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    TalonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    LeftTalonFXConfigurator.apply(TalonFXConfigs);
    //INVERTS ONE MOTOR
    m_MotorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
    //leftMotor.setInverted(false);
    RightTalonFXConfigurator.apply(m_MotorOutputConfigs);
    //sets to brake mode
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake); 
/* User can change the configs if they want, or leave it empty for factory-default */
    /*  invert in software, 
    encoder.setInverted; */
   // strict followers ignore the leader's invert and use their own
    rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));
    // creates PID values for motor (hopefully)
    var slot0Configs = new Slot0Configs();
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0Configs.kP = ElevatorConstants.kP;
    slot0Configs.kI = ElevatorConstants.kI;
    slot0Configs.kD = ElevatorConstants.kD;
   //m_voltagePosition.Slot = 0;
    leftMotor.getConfigurator().apply(slot0Configs);
  }
  public void runToPosition(double setpoint) {
    leftMotor.setControl(m_request.withPosition(setpoint));
}
public void run(double setpoint) {
  leftMotor.set(setpoint);
  }
public double getPos() {
  return encoder.getPosition().getValueAsDouble();
}
  @Override
  public void periodic() {
    SmartDashboard.putNumber("getPos",getPos());

  }
}
