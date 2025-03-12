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
    private SparkMax algaeMotor;
    private SparkMax armMotor;
    private SparkMaxConfig algaeConfig;
    private SparkMaxConfig armConfig;

    public AlgaeIntake(int algaeID, int armID) {

        algaeMotor = new SparkMax(algaeID, MotorType.kBrushless);
        armMotor = new SparkMax(armID, MotorType.kBrushless);
        // leftAlgaeMotor.restoreFactoryDefaults();
        // rightAlgaeMotor.restoreFactoryDefaults();

        // pid = leftAlgaeMotor.getPIDController();

        // leftAlgaeMotor.restoreFactoryDefaults();
        // rightAlgaeMotor.restoreFactoryDefaults();
        algaeConfig
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
        algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armConfig
        .apply(algaeConfig)
        .smartCurrentLimit(140)
        .inverted(true);
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
       
    }

public void runAlgae(double speed) {
algaeMotor.set(speed);
}
// public void elevatorToPosition(double setPoint) {
// // Set the setpoint of the PID controller in raw position mode
// m_controller.setReference(setPoint, ControlType.kPosition);
// }
public void algaeStop() {
algaeMotor.set(0);
}
public void runArm(double setpoint) {
armMotor.set(setpoint);
}

public void stopArm() {
    armMotor.set(0);
    }

@Override
public void periodic() {}
}