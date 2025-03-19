// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunWrist extends Command {
  /** Creates a new RunWrist. */
   private final Wrist wrist;
  private double setpoint;
  // private boolean end;
  public RunWrist(Wrist wristInput, double speed) {
    wrist = wristInput;
    setpoint = speed; 
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // end = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.run(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.run(0); // change speed later for later 
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (wrist.getPos() <= -0.001) {
      return true;
    }
    return false;
  }
}
