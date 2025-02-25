// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPosition extends Command {
  /** Creates a new Climber_up. */
  private final Elevator elevator;
  private double setpoint;
  private boolean end;
  public ElevatorPosition(Elevator elevate, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = elevate;
    setpoint = target;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runToPosition(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((MathUtil.isNear(setpoint, elevator.getPos(), 0.01))||(elevator.getPos() >= 1)||(elevator.getPos() <= -1)||(elevator.getOptical()==true)) {
      return true;
    } else {
      return false;
    }  }
}
