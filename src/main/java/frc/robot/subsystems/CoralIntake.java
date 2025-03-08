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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private SparkMax armMotor;
  private SparkMaxConfig armConfig; 
  private SparkMax intakeMotor;
  private SparkMaxConfig intakeConfig;
  private SparkClosedLoopController armPID;
  public CoralIntake(int intakeID, int armID) {
    intakeMotor = new SparkMax(intakeID, MotorType.kBrushless);
    intakeConfig
    .inverted(false)
    .smartCurrentLimit(40)
    .voltageCompensation(12.6)
    .idleMode(IdleMode.kBrake);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armMotor = new SparkMax(armID, MotorType.kBrushless);
    armConfig
    .inverted(false)
    .smartCurrentLimit(40)
    .voltageCompensation(12.6)
    .idleMode(IdleMode.kBrake);
    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
    //.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .p(0)
    .i(0)
    .d(0)
    .outputRange(-1, 1);
    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armPID = armMotor.getClosedLoopController();
  }
  
  public void runCoral(double speed) {
    intakeMotor.set(speed);
  }
  
  public void coralStop() {
    intakeMotor.set(0);
  }
  
  public void intakeToPosition(double setpoint) {
   armPID.setReference(setpoint, ControlType.kPosition);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
