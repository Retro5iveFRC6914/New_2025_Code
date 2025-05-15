// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevator extends Command {
  /** Creates a new RunElevator. */
   private final Elevator elevator;
  private double setpoint;
  // private boolean end;
  public RunElevator(Elevator elevate, double speed) {
    elevator = elevate;
    setpoint = speed; 
    addRequirements(elevator);
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
    elevator.run(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.run(0); // change speed later for later 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((elevator.getPos() <= -0.2 ) ||(elevator.getPos() >= 4.32)) {
      System.out.println("Elevator is at max");
      return true;
    }
    return false;
  }
}
