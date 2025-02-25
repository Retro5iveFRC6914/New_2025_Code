// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.LimelightHelpers;
// import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
// import edu.wpi.first.wpilibj2.command.Command;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class LimelightOrient extends Command {
//   /** Creates a new LimelightOrient. */
//   private CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
//   private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
//   private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
//   private double TX;
//   private double TY;
//   private double TA;
//   private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
//       .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
//       .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
//                                                                // driving in open loop

//   public LimelightOrient() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(drivetrain);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
// TX = LimelightHelpers.getTX("limelight-thehulk");
// TY = LimelightHelpers.getTY("limelight-thehulk");
// TA = LimelightHelpers.getTA("limelight-thehulk");
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
// TX = LimelightHelpers.getTX("limelight-thehulk");
// TY = LimelightHelpers.getTY("limelight-thehulk");
// TA = LimelightHelpers.getTA("limelight-thehulk");

//     drivetrain.applyRequest(() -> drive
// .withVelocityX(-TX * 0.1) // Drive forward with
// // negative Y (forward)
// .withVelocityY(-TY * 0.1) // Drive left with negative X (left)
// .withRotationalRate(-TA * 0.1)) // Drive counterclockwise with negative X (left)
// ;
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     drivetrain.applyRequest(() -> drive
// .withVelocityX(0) // Drive forward with
// // negative Y (forward)
// .withVelocityY(0)// Drive left with negative X (left)
// .withRotationalRate(0)) // Drive counterclockwise with negative X (left)
// ;
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//   if (TX == 0 && TY == 0 && TA == 0) {
//     return true;
//   } else {
//     return false;
//   }
// }
// }
