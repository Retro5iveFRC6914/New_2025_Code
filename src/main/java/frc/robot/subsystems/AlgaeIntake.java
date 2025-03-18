package frc.robot.subsystems;



import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;




import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    //Creates TalonFX objects for motors 
    private final TalonFX intake;
    private final TalonFX arm;
    //Creates CANcoder object for the CANcoder encoder
    // private final DigitalInput optical;
    //Creates MotorOutputConfigs object, and resets motor output configs
    private final MotorOutputConfigs m_MotorOutputConfigs = new MotorOutputConfigs();
    //Creates CurrentLimitsConfigs object, and resets current limits configs
   // private final DutyCycleOut m_output = new DutyCycleOut(0);
    // private final CANcoder encoder;
    public AlgaeIntake(int algaeID, int armID/* , int CANcoderID*/) {
//declares the motors as motors with the ID's from the arguments
    intake = new TalonFX(algaeID);
    arm = new TalonFX(armID);
    //throughBore = new Encoder(1,2);
    // encoder = new CANcoder(CANcoderID);
    // optical = new DigitalInput(3);
    TalonFXConfiguration algaeConfig = new TalonFXConfiguration();
   // creates current limit for motors
    algaeConfig.CurrentLimits.withSupplyCurrentLimit(140);

    intake.getConfigurator().apply(algaeConfig);

    arm.getConfigurator().apply(algaeConfig);
    //INVERTS ONE MOTOR
    m_MotorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
    //intake.setInverted(false);
    arm.getConfigurator().apply(m_MotorOutputConfigs);
    //sets to brake mode
    intake.setNeutralMode(NeutralModeValue.Brake);
    arm.setNeutralMode(NeutralModeValue.Brake); 
/* User can change the configs if they want, or leave it empty for factory-default */
   // strict followers ignore the leader's invert and use their own
    // arm.setControl(new StrictFollower(intake.getDeviceID()));
    // intake.getConfigurator().apply(algaeConfig);
    // creates PID values for motor (hopefully)
//     var slot0Configs = new Slot0Configs();
//     slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
//     slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
//     slot0Configs.kG = ElevatorConstants.kG;
//     slot0Configs.kS = ElevatorConstants.kS;
//     slot0Configs.kV = ElevatorConstants.kV;
//     slot0Configs.kA = ElevatorConstants.kA;
//     slot0Configs.kP = ElevatorConstants.kP;
//     slot0Configs.kI = ElevatorConstants.kI;
//     slot0Configs.kD = ElevatorConstants.kD;
//    //m_voltagePosition.Slot = 0;
//     intake.getConfigurator().apply(slot0Configs);
  }
//   public void runToPosition(double setpoint) {
//     intake.setControl(m_request.withPosition(setpoint));
// }
public void runAlgae(double setpoint) {
  intake.set(setpoint);
  }

public void algaeStop() {
  intake.set(0);
}

public void runArm(double setpoint) {
    arm.set(setpoint);
}
public void stopArm() {
    arm.set(0);
}
@Override
public void periodic() {}
}