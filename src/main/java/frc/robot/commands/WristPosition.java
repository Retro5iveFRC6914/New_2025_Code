// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristPosition extends Command {
  /** Creates a new WristPosition. */
  private Wrist wrist;
  private double target;
  // private boolean end;
  public WristPosition(Wrist ctreWrist, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    wrist = ctreWrist;
    target = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setToPosition(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     if ((MathUtil.isNear(target, wrist.getPos(), 0.005))||(wrist.getPos() >= 0.46)||(wrist.getPos() <= -0.001)) {
      return true;
    } else {
      return false;
    }  
  }
}
