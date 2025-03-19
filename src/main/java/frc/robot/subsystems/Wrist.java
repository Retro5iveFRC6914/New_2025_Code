// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import static edu.wpi.first.units.Units.Hertz;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.TalonFXSConfiguration;
// import com.ctre.phoenix6.hardware.TalonFXS;
// import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.MotorArrangementValue;

import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
// import com.revrobotics.AbsoluteEncoder;

// import javax.swing.text.Position;


// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.StrictFollower;
// import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.CANcoderConfigurator;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.controller.PIDController;
public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final TalonFX wrist;
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  public Wrist(int wristID) {
    // Establishes wrist as a TalonFXS as a motor with an ID of the int parameter "wristID"
    wrist = new TalonFX(wristID);
    //Establishes throughBore as an absolute encoder with an ID of 3
    //PIDController tolerance

    // Creates a new TalonFXS configuration
    TalonFXConfiguration wristConfigure = new TalonFXConfiguration();
    // Allows the configuration to know it is a Minion
    // Sets a current limit of 40 amps
    wristConfigure.CurrentLimits.withSupplyCurrentLimit(40);
    // Sets the wrist to be counter clockwise positive
    wristConfigure.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    // Makes the motor into brake mode
    wristConfigure.MotorOutput.withNeutralMode(NeutralModeValue.Brake); 
    // Sets the motor's gravity type to an arm type
    //wristConfigure.ExternalFeedback.withRotorToSensorRatio();
    wristConfigure.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    wristConfigure.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
    wristConfigure.Feedback.withSensorToMechanismRatio(125);
    // PID values for the wrist motor
    wristConfigure.Slot0.kP = 20;
    wristConfigure.Slot0.kI = 0;
    wristConfigure.Slot0.kD = 0;
    // Applies the motor's configurations to the motor
    wrist.getConfigurator().apply(wristConfigure);
    wrist.setPosition(0);
  }

  public void setToPosition(double setpoint) {
  wrist.setControl(m_request.withPosition(setpoint));
  }

  // public void runPID(double setpoint) {
  //   wrist.set(m_wristPID.calculate(throughBore.get(), setpoint));
  // }

  public void run(double setpoint) {
    wrist.set(setpoint);
  }
  public void stop() {
    wrist.set(0);
  }

  public double getPos() {
    return wrist.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Position", getPos());
  }
}
