
package frc.robot.subsystems;

 //import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkMax intakeMotor;

  private final SparkMaxConfig intakeConfig;

  private final SparkClosedLoopController intakeController;

  private final RelativeEncoder intakeCoder;
  
  
  
    public IntakeSubsystem() {
  
  
      //DEVICE ID WILL CHANGE WHEn ROBOT.
      intakeMotor = new SparkMax(14, MotorType.kBrushless);
      intakeConfig = new SparkMaxConfig();    
    
      // IdleMode is brake vs coast. Brake stops when it stops recieving power, coast will let it coast.
      intakeConfig.idleMode(IdleMode.kCoast);
      
      // This assignment gets the encoder from the motor object defined earlier. A RelativeEncoder is an object created with each CANSparkMax controller.
      intakeCoder = intakeMotor.getEncoder();
      intakeController = intakeMotor.getClosedLoopController();
  
      Preferences.initDouble("IntakeP", 0.0006);
      Preferences.initDouble("IntakeI", 0);
      Preferences.initDouble("IntakeD", 0);
  
      intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(Preferences.getDouble("IntakeP", 0.0006))
      .i(Preferences.getDouble("IntakeI", 0))
      .d(Preferences.getDouble("IntakeD", 0))
      .feedForward.kV(0.00018);
      
      //11111cchbbbbbbbbbbbbbbbbbbbbbbbbbbyyyb333333eeeeeeeeeeeeeeeeeeeee BUNNY CODE!! `
  
      intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  public void lowAmps() {
    intakeConfig.smartCurrentLimit(20);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void highAmps() {
    intakeConfig.smartCurrentLimit(80);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Simple function to spin the intake motor at the parameter speed. 
   * @param speed Speed between -1.0 and 1.0.
   */
  public void spinIntake(double speed) {
    // set(speed) is the simple way to set speed for a SparkMAX. It differs slightly from a Talon - see another subsystem for that.
    // intakeController.setReference(speed, ControlType.kVelocity);
    intakeMotor.set(speed);
  }

  public void spinIntakeRPM(double rpm) {
    intakeController.setSetpoint(rpm, ControlType.kVelocity);
  }

  public double getIntakeVelocity(){
    return intakeCoder.getVelocity();
  }
  
  /**
   * Simple function to stop the intake. It is good to have this as opposed to running spinIntake(0), because it ensures there is no error.
   */
  public void stopIntake() {
   //intakeController.setReference(0, ControlType.kVelocity);
   intakeMotor.set(0);
  }
  
  /**
   * Periodic function standard to all subsystems.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake RPM", getIntakeVelocity());
  }
}