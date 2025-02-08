// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.*;
import frc.robot.commands.ElevatorPosition;
  /*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final Elevator m_elevator = new Elevator(ElevatorConstants.kLeftID, ElevatorConstants.kRightID);
  public final Intake m_intake = new Intake(IntakeConstants.kIntakeID, IntakeConstants.kSensorDIOPort);
  public final Shooter m_shooter = new Shooter(ShooterConstants.kTopID, ShooterConstants.kBottomID);
  public final Climber m_climber = new Climber(ClimberConstants.kLeftID, ClimberConstants.kRightID);

  /* Controllers */

    // The driver's controller
  CommandXboxController operatorXboxController = new CommandXboxController(1);
  CommandXboxController driverXboxController = new CommandXboxController(0);
    
//auto chooser creation
  private final SendableChooser<Command> autoChooser;


  public RobotContainer(){
    // Register Named Commands
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    // Another option that allows you to specify the default auto by its name
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure default commands
    // Subsystem Default Commands
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.XboxController} or {@link XboxController}), and then calling
   * passing it to a
   * {@link XboxButton}.
   */
  private void configureButtonBindings() {
  operatorXboxController.a().onTrue(new ElevatorPosition(m_elevator, 0));
  operatorXboxController.b().onTrue(new ElevatorPosition(m_elevator, 0));
  operatorXboxController.x().onTrue(new ElevatorPosition(m_elevator, 0));
  operatorXboxController.y().onTrue(new ElevatorPosition(m_elevator, 0));
  operatorXboxController.leftTrigger().whileTrue(new RunAlgae(m_intake, 1));
  operatorXboxController.rightTrigger().whileTrue(new RunAlgae(m_intake, -1));
  operatorXboxController.leftBumper().whileTrue(new RunCoral(m_intake, 1));
  operatorXboxController.rightBumper().whileTrue(new RunCoral(m_intake, -1));

  driverXboxController.leftTrigger().whileTrue(new RunClimber(m_climber, 0));
  driverXboxController.rightTrigger().whileTrue(new RunClimber(m_climber, 0));
  }
    
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}