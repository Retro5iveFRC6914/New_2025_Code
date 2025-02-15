// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  private Intake intake;
  private double topSpeed;
  private double bottomSpeed;

  private boolean end;
  /** Creates a new RunIntake. */
  public RunIntake(Intake launchingDevice, double topPercent, double bottomPercent) {
    intake = launchingDevice;
    topSpeed = topPercent;
    bottomSpeed = bottomPercent;
    end = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launchingDevice);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("init RunIntake");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runOpenLoop(topSpeed, bottomSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runOpenLoop(0,0);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}