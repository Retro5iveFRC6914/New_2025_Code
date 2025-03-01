// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final TalonFXS wristMotor = new TalonFXS(0, "canivore");
  public Wrist(int wristID) {
    TalonFXSConfiguration toConfigure = new TalonFXSConfiguration();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
