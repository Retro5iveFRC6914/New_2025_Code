package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class AlgaeIntake extends SubsystemBase {
    private SparkMax leftAlgaeMotor;
    private SparkMax rightAlgaeMotor;
    private SparkMaxConfig leftAlgaeConfig;
    private SparkMaxConfig rightAlgaeConfig;
    private RelativeEncoder encoder;
    private DigitalInput limitSwitch;
    SparkClosedLoopController m_controller = leftAlgaeMotor.getClosedLoopController();

    public AlgaeIntake(int leftID, int rightID, int coralID) {

        leftAlgaeMotor = new SparkMax(leftID, MotorType.kBrushless);
        rightAlgaeMotor = new SparkMax(rightID, MotorType.kBrushless);
        // leftAlgaeMotor.restoreFactoryDefaults();
        // rightAlgaeMotor.restoreFactoryDefaults();

        // pid = leftAlgaeMotor.getPIDController();

        // leftAlgaeMotor.restoreFactoryDefaults();
        // rightAlgaeMotor.restoreFactoryDefaults();
        leftAlgaeConfig
        .inverted(false)
        .smartCurrentLimit(40)
        .voltageCompensation(12.6)
        .idleMode(IdleMode.kBrake);
    //  leftAlgaeConfig.closedLoop
    //  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //  .p(0)
    //  .i(0)
    //  .d(0)
    //  .outputRange(0, 1);
        leftAlgaeMotor.configure(leftAlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightAlgaeConfig
        .apply(leftAlgaeConfig)
        .follow(leftID)
        .inverted(true);
        rightAlgaeMotor.configure(rightAlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
       
    }

public void runAlgae(double speed) {
leftAlgaeMotor.set(speed);
}
// public void elevatorToPosition(double setPoint) {
// // Set the setpoint of the PID controller in raw position mode
// m_controller.setReference(setPoint, ControlType.kPosition);
// }
public void algaeStop() {
leftAlgaeMotor.set(0);
}


@Override
public void periodic() {}
}