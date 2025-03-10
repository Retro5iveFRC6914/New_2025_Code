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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.AlgaeIntake;

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
  public final Elevator m_elevator = new Elevator(ElevatorConstants.kLeftID, ElevatorConstants.kRightID, ElevatorConstants.kEncoderID);
  public final AlgaeIntake m_algaeIntake = new AlgaeIntake(IntakeConstants.kLeftID, IntakeConstants.kRightID, IntakeConstants.kCoralID);
  public final Climber m_climber = new Climber(ClimberConstants.kLeftID, ClimberConstants.kRightID);
  public final CoralIntake m_coralIntake = new CoralIntake(IntakeConstants.kCoralID, IntakeConstants.kCoralArmID);

  /* Controllers */

    // The driver's controller
  CommandXboxController operatorXboxController = new CommandXboxController(1);
  CommandXboxController driverXboxController = new CommandXboxController(0);
    
//auto chooser creation
  private final SendableChooser<Command> autoChooser;


  public RobotContainer(){
    // Register Named Commands
        // NamedCommands.registerCommand("intake", intake);
    // NamedCommands.registerCommand("homePort", new HomeClimber(m_portClimber));
    // NamedCommands.registerCommand("homeStarboard", new HomeClimber(m_starboardClimber));
  // elevator levels 
    NamedCommands.registerCommand("elevatorToL1", new ElevatorPosition(m_elevator, 0).withTimeout(2)); // add the pos later for all Levels
    NamedCommands.registerCommand("elevatorToL2", new ElevatorPosition(m_elevator, 0).withTimeout(2));
    NamedCommands.registerCommand("elevatorToL3", new ElevatorPosition(m_elevator, 0).withTimeout(2));
    NamedCommands.registerCommand("elevatorToL4", new ElevatorPosition(m_elevator, 0).withTimeout(2));
   // stuff to grab and score coral
    NamedCommands.registerCommand("grabCoral", new RunCoral(m_coralIntake, 1).withTimeout(1.5));
    NamedCommands.registerCommand("scoreCoral", new RunCoral(m_coralIntake, -1).withTimeout(1.5));
// stuff to grab and score algae 
    NamedCommands.registerCommand("grabAlgae", new RunAlgae(m_algaeIntake, 1).withTimeout(1.5));
    NamedCommands.registerCommand("scoreAlgae", new RunAlgae(m_algaeIntake, -1).withTimeout(1.5));



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
// L1 Pos
  operatorXboxController.a().onTrue(new ElevatorPosition(m_elevator, 0));
  // L2 Pos
  operatorXboxController.b().onTrue(new ElevatorPosition(m_elevator, 0));
 // L3 Pos
  operatorXboxController.x().onTrue(new ElevatorPosition(m_elevator, 0));
 // L4 Pos
  operatorXboxController.y().onTrue(new ElevatorPosition(m_elevator, 0));
 // Runs intake for Algae in 
  operatorXboxController.leftTrigger().whileTrue(new RunCommand(() -> m_algaeIntake.runAlgae(1), m_algaeIntake))
  .onFalse(new RunCommand(() -> m_algaeIntake.runAlgae(0), m_algaeIntake));
// Runs intake for Algae out 
  operatorXboxController.rightTrigger().whileTrue(new RunCommand(() -> m_algaeIntake.runAlgae(-1), m_algaeIntake))
  .onFalse(new RunCommand(() -> m_algaeIntake.algaeStop(), m_algaeIntake));
// Runs the intake for Coral in 
  operatorXboxController.leftBumper().whileTrue(new RunCommand(() -> m_coralIntake.runCoral(1), m_coralIntake))
  .onFalse(new RunCommand(() -> m_coralIntake.coralStop(), m_coralIntake));
// Runs the intake for Coral out 
  operatorXboxController.rightBumper().whileTrue(new RunCommand(() -> m_coralIntake.runCoral(-1), m_coralIntake))
  .onFalse(new RunCommand(() -> m_coralIntake.runCoral(0), m_coralIntake));
// Runs the Climber to go up 
  driverXboxController.leftTrigger().whileTrue(new RunCommand(() -> m_climber.runClimber(1), m_climber))
  .onFalse(new RunCommand(() -> m_climber.stop(), m_climber));
// Runs the Climber to go down
  driverXboxController.rightTrigger().whileTrue(new RunCommand(() -> m_climber.runClimber(-1), m_climber))
  .onFalse(new RunCommand(() -> m_climber.stop(), m_climber)); 
// buttons for Limelight need to convert later to 2025 version
//   driverXboxController.a().whileTrue(drivetrain.applyRequest(() -> drive
// .withVelocityX(-LimelightHelpers.getTX("limelight-thehulk") * 0.1) // Drive forward with
// // negative Y (forward)
// .withVelocityY(-LimelightHelpers.getTY("limelight-thehulk") * 0.1) // Drive left with negative X (left)
// .withRotationalRate(-LimelightHelpers.getTA("limelight-thehulk") * 0.1) // Drive counterclockwise with negative X (left)
// ));

}
    
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}