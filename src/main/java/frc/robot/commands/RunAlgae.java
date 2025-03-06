// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunAlgae extends Command {
  /** Creates a new RunAlgae. */
   private final AlgaeIntake intaker;
  private double setpoint;
  private boolean end;
  public RunAlgae(AlgaeIntake intake, double speed) {
    intaker = intake;
    setpoint = speed; 
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intaker.runAlgae(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intaker.runAlgae(0); // change speed later for later 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
