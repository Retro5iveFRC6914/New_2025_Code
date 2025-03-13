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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
  /** Creates a new phoenix. */  
  //Creates TalonFX objects for motors 
    private final TalonFX climber;
    //Creates MotorOutputConfigs object, and resets motor output configs
    private final MotorOutputConfigs m_MotorOutputConfigs = new MotorOutputConfigs();
    //Creates CurrentLimitsConfigs object, and resets current limits configs
    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();
   // private final DutyCycleOut m_output = new DutyCycleOut(0);

    public Climber(int climberID) {
//declares the motors as motors with the ID's from the arguments
    climber = new TalonFX(climberID);
    TalonFXConfiguration TalonFXConfigs = new TalonFXConfiguration();
    TalonFXConfigurator climberConfigurator = climber.getConfigurator();
    // sets to factory default
    climberConfigurator.apply(new TalonFXConfiguration());
    //declares new cancoder       
    // creates current limit for motors
    m_currentLimits.SupplyCurrentLimit = 140;
    TalonFXConfigs.CurrentLimits = m_currentLimits;
    //set current limit for left motor
    climberConfigurator.apply(m_currentLimits); 
    //set current limit for right motor
    //inverts one motor to create rotation
    climberConfigurator.apply(TalonFXConfigs);
    //INVERTS ONE MOTOR
    m_MotorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);
    //leftMotor.setInverted(false);
    climberConfigurator.apply(m_MotorOutputConfigs);
    //sets to brake mode
    climber.setNeutralMode(NeutralModeValue.Brake);
/* User can change the configs if they want, or leave it empty for factory-default */
    /*  invert in software, 
    encoder.setInverted; */
   // strict followers ignore the leader's invert and use their own
    // creates PID values for motor (hopefully)
   
   //m_voltagePosition.Slot = 0;
}
public void run(double setpoint) {
    climber.set(setpoint);
}

public void stop() {
    climber.set(0);
}
}  
// // public void runToPosition(double setpoint) {
// //     leftMotor.setControl(m_request.withPosition(setpoint));
// // }
// public void runClimber(double setpoint) {
//   leftMotor.set(setpoint);
//   }
//   public void stop() {
//     leftMotor.set(0);
//     }

 
  
// }
