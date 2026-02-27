package frc.robot.subsystems;

//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  ;
  private final SparkMax leftFlywheelMotor;
  private final SparkMax rightFlywheelMotor;

  private final SparkMaxConfig leftFlywheelConfig;
  private final SparkMaxConfig rightFlywheelConfig;

  // private final SparkClosedLoopController intakeController;
  private final SparkClosedLoopController flywheelController;

  private final RelativeEncoder leftFlywheelCoder;
  private final RelativeEncoder rightFlywheelCoder;

  private final DigitalInput shooterBeamBreak;

  public ShooterSubsystem() {

    // DEVICE ID WILL CHANGE WHEn ROBOT.
    leftFlywheelMotor = new SparkMax(8, MotorType.kBrushless);
    rightFlywheelMotor = new SparkMax(9, MotorType.kBrushless);

    leftFlywheelConfig = new SparkMaxConfig();
    rightFlywheelConfig = new SparkMaxConfig();

    // intakeController = spinMotor.getClosedLoopController();
    shooterBeamBreak = new DigitalInput(2);

    // IdleMode is brake vs coast. Brake stops when it stops recieving power, coast
    // will let it coast.
    leftFlywheelConfig.idleMode(IdleMode.kBrake);
    rightFlywheelConfig.idleMode(IdleMode.kBrake);
    rightFlywheelConfig.follow(leftFlywheelMotor, true);

    // leftFlywheelConfig.inverted(true);

    leftFlywheelConfig.smartCurrentLimit(40);
    rightFlywheelConfig.smartCurrentLimit(40);

    // This assignment gets the encoder from the motor object defined earlier. A
    // RelativeEncoder is an object created with each CANSparkMax controller.

    rightFlywheelCoder = rightFlywheelMotor.getEncoder();
    leftFlywheelCoder = leftFlywheelMotor.getEncoder();

    leftFlywheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SmartDashboard.getNumber("IntakeP", 0.0004))
        .i(SmartDashboard.getNumber("IntakeI", 0))
        .d(SmartDashboard.getNumber("IntakeD", 0))
        .feedForward.kV(5.0 / 5767);

    // rightFlywheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .p(SmartDashboard.getNumber("IntakeP", 0.001))
    //     .i(SmartDashboard.getNumber("IntakeI", 0))
    //     .d(SmartDashboard.getNumber("IntakeD", 0)).feedForward.kV(5.0 / 5767);


    flywheelController = leftFlywheelMotor.getClosedLoopController();

    rightFlywheelMotor.configure(rightFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFlywheelMotor.configure(leftFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // A digital input is the slots 0-9 on the RoboRIO in the "DIO" area. You plug
    // in limit switches into here normally. Essentially, this declaration points to
    // the number 9 slot on the DIO.
  }

  public boolean isIntakeBeamBreakTripped() {
    return shooterBeamBreak.get();
  }

  public void spinShooter(double speed) {
    // set(speed) is the simple way to set speed for a SparkMAX. It differs slightly
    // from a Talon - see another subsystem for that.
    // intakeController.setReference(speed, ControlType.kVelocity);
    leftFlywheelMotor.set(speed);
  }

  public void spinShooterSpeed(double rpm){
    flywheelController.setSetpoint(rpm, ControlType.kVelocity);
  }

  public double getShooterSpeed() {
    return leftFlywheelCoder.getVelocity();
  }

  /**
   * Simple function to stop the intake. It is good to have this as opposed to
   * running spinIntake(0), because it ensures there is no error.
   */
  public void stopShooter() {
    // intakeController.setReference(0, ControlType.kVelocity);
    leftFlywheelMotor.set(0);

  }
  // ynmtocc4 MOrE BUNNY CODE!!!!!

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterRPM", getShooterSpeed());
  }
}