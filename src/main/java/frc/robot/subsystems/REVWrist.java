// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class REVWrist extends SubsystemBase {
  /** Creates a new REVWrist. */
  private SparkMax wristMotor;
  private SparkMaxConfig wristConfig;
  private SparkClosedLoopController pidController;
  private DutyCycleEncoder encoder;

  public REVWrist(int wristID) {
    wristMotor = new SparkMax(wristID, MotorType.kBrushless);
    pidController = wristMotor.getClosedLoopController();
    encoder = new DutyCycleEncoder(3);
    wristConfig
    .inverted(false)
    .smartCurrentLimit(40)
    .voltageCompensation(12.6)
    .idleMode(IdleMode.kBrake);
    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
    .p(0)
    .i(0)
    .d(0)
    .outputRange(-1, 1);
    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void runWrist(double speed) {
    wristMotor.set(speed);
  }
  
  public void wristStop() {
    wristMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
