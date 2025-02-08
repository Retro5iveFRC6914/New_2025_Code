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

public class Intake extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private SparkMaxConfig leftConfig;
    private SparkMaxConfig rightConfig;
    private RelativeEncoder encoder;
    private DigitalInput limitSwitch;
    SparkClosedLoopController m_controller = leftMotor.getClosedLoopController();

    public Intake(int leftID, int rightID) {

        leftMotor = new SparkMax(leftID, MotorType.kBrushless);
        rightMotor = new SparkMax(rightID, MotorType.kBrushless);
        // leftMotor.restoreFactoryDefaults();
        // rightMotor.restoreFactoryDefaults();

        encoder = leftMotor.getEncoder();
        // pid = leftMotor.getPIDController();

        // leftMotor.restoreFactoryDefaults();
        // rightMotor.restoreFactoryDefaults();
        leftConfig
        .inverted(false)
        .smartCurrentLimit(40)
        .voltageCompensation(12.6)
        .idleMode(IdleMode.kBrake);
    //  leftConfig.closedLoop
    //  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //  .p(0)
    //  .i(0)
    //  .d(0)
    //  .outputRange(0, 1);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightConfig
        .apply(leftConfig)
        .follow(leftID)
        .inverted(true);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

public void runClimber(double speed) {
leftMotor.set(speed);
}
// public void elevatorToPosition(double setPoint) {
// // Set the setpoint of the PID controller in raw position mode
// m_controller.setReference(setPoint, ControlType.kPosition);
// }
public void stop() {
leftMotor.set(0);
}
public double getPos() {
        return encoder.getPosition();
}
@Override
public void periodic() {
        SmartDashboard.putNumber("Climber Position", getPos());
}
}