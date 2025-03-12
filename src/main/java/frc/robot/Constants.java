package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
         kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

//* arm constants
            
    }
    public static final class ElevatorConstants {
        public static final int kLeftID = 9;
        public static final int kRightID = 10;
        public static final int kCANCoderID = 15;
       
    
        //these 2 should be the same
        public static final double kIntakePos = 0.06;
        public static final double kAutoSpeakerPos = 0.065;
        public static final double kTeleOpSpeakerPos = 0.0625;
        public static final double kFrontAmpPos = 0.15;
    
        // 3m position
       
        public static final double kG = 0; //TODO changed this
        public static final double kS = 0; //TODO changed this 
        public static final double kV = 0; //TODO changed this 
        public static final double kA = 0; //TODO changed this 

        public static final double kP = 0; //TODO changed this 
        public static final double kI = 0; //TODO changed this
        public static final double kD = 0;  //TODO changed this
        public static final double kIz = 0; //TODO changed this 
        public static final double kFF = 0; //TODO changed this 
        public static final double kMaxOutput = 0.7; 
        public static final double kMinOutput = -0.7;
        public static final double kMaxAccel = 0.18;
        public static final double kMaxVel = 0.85;
    
        public static final double kTolearance = 0.002;
        public static final double kManualSpeed = 0.3;

//*  shooter constants

      }
    
//* Climber constants 
    public static final class ClimberConstants {
    public static final int kLeftID = 23;
    public static final int kRightID = 24;
    public static final double kP = 0.1; 
    public static final double kI = 1e-4;
    public static final double kD = 1; 
    public static final double kIz = 0; 
    public static final double kFF = 0; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;

    //public static final double kHomeSpeed = -0.4;
    
    public static final double kUpperLimit = 10;
    public static final double kLowerLimit = 0;
    public static final double kManualSpeed = 0.3;
}

// Cannot remember if the Extreme 3D PRO is 0 index or not.
public static final class DriverConstants {
  public static final double kDefaultSpeed = 0.8;
  public static final int kSetX = 5;
  public static final int kIntake = 0;
  public static final int kHoldArmDown = 6;
  public static final int kAutoIntake = 3; 
  public static final int kAutoAmp = 4;
  public static final int kSelfDestruct = 9;
  public static final int kTurbo = 1;

//* intake constants

}
 public static final class IntakeConstants {
            public static final int kAlgaeID = 11;
            public static final int kAlgaeArmID = 12;
            public static final int kCoralWristID = 13;
            public static final int kCoralIntakeID = 14;
            public static final double kIntakeSpeed = 0.4;
            public static final double kFeedSpeed = 3083;
            public static final double kReverseSpeed = 1;
            public static final double kCompenstion = 1.83;
        
            public static final double kP = 6e-5;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kFF = 0.000015;
            public static final double kIz = 0;
            public static final double kMinOutput = -1;
            public static final double kMaxOutput = 1;
            public static final double kMaxRPM = 11000;    
          }

          public static final class OIConstants {
            public static final int kDriverControllerPort = 0;
            public static final double kDriveDeadband = 0.1;
            public static final int kOperatorControllerPort = 1;
             }

             // operator constants

            public static final class OperatorConstants {
                public static final double kManoeuvreSpeed = 0.4; 
            }
    }

  
