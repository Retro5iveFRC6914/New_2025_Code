// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// falcsons instead of neos so change everything 
package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
// import com.revrobotics.AbsoluteEncoder;

// import javax.swing.text.Position;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
// import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.CANcoderConfigurator;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.DutyCycleOut;

/*import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;*/

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ElevatorConstants;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
  /** Creates a new phoenix. */  
  //Creates TalonFX objects for motors 
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    //Creates CANcoder object for the CANcoder encoder
    // private Encoder throughBore;
    // private final DigitalInput optical;
    //Creates MotorOutputConfigs object, and resets motor output configs
    private final MotorOutputConfigs m_rightMotorOutputConfigs = new MotorOutputConfigs();
    private final MotorOutputConfigs m_leftMotorOutputConfigs = new MotorOutputConfigs();

    //Creates CurrentLimitsConfigs object, and resets current limits configs
   // private final DutyCycleOut m_output = new DutyCycleOut(0);
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    // private final CANcoder encoder;
    // private final FeedbackConfigs feedback;
    public Elevator(int leftID, int rightID/* , int CANcoderID*/) {
//declares the motors as motors with the ID's from the arguments
    leftMotor = new TalonFX(leftID);
    rightMotor = new TalonFX(rightID);
    //throughBore = new Encoder(1,2);
    // encoder = new CANcoder(CANcoderID);
    // optical = new DigitalInput(3);
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    // feedback = new FeedbackConfigs();
   // creates current limit for motors
    elevatorConfig.CurrentLimits.withSupplyCurrentLimit(140);

    leftMotor.getConfigurator().apply(elevatorConfig);

    rightMotor.getConfigurator().apply(elevatorConfig);

    //sets to brake mode
/* User can change the configs if they want, or leave it empty for factory-default */
   // strict followers ignore the leader's invert and use their own
    rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));
    elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    elevatorConfig.Feedback.SensorToMechanismRatio = 15;
    // creates PID values for motor (hopefully)
    elevatorConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static);
    elevatorConfig.Slot0.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // slot0Configs.kG = ElevatorConstants.kG;
    elevatorConfig.Slot0.kP = 0.07;
    // slot0Configs.kI = ElevatorConstants.kI;
    // slot0Configs.kD = ElevatorConstants.kD;
  //  m_voltagePosition.Slot = 0;
    leftMotor.getConfigurator().apply(elevatorConfig);
    rightMotor.getConfigurator().apply(elevatorConfig);
        //INVERTS ONE MOTOR
        m_rightMotorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);
        m_rightMotorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
        //leftMotor.setInverted(false);    
        rightMotor.getConfigurator().apply(m_rightMotorOutputConfigs);
        //invert left motor to clockwise positive
        m_leftMotorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
        m_leftMotorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

        leftMotor.getConfigurator().apply(m_leftMotorOutputConfigs);
    leftMotor.setPosition(0);
  }
  public void runToPosition(double setpoint) {
    leftMotor.setControl(m_request.withPosition(setpoint));
}
public void run(double setpoint) {
  leftMotor.set(setpoint);
  }

public void stop() {
  leftMotor.set(0);
}
public void zeroPosition() {
  leftMotor.setPosition(0);
}
public double getPos() {
  // return throughBore.get();
  return leftMotor.getPosition().getValueAsDouble();
}
// public boolean getOptical() {
//   return !optical.get();
// }
  @Override
  public void periodic() {
   /*  if(getOptical() == true) {
      throughBore.reset();
    } */
    // SmartDashboard.putBoolean("Zero Sensor", getOptical());
    SmartDashboard.putNumber("Elevator Position", getPos());

  }
}
