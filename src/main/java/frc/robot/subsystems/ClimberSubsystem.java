
package frc.robot.subsystems;

 //import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkMax climberMotor;

  private final SparkMaxConfig climberConfig;

  private final SparkClosedLoopController climberController;

  private final RelativeEncoder climberCoder;
  
  
  
    public ClimberSubsystem() {
  
  
      //CHANGE CLIMB MOTOR ID!!!!!!!
      climberMotor = new SparkMax(99, MotorType.kBrushless);
      climberConfig = new SparkMaxConfig();    
    
      // IdleMode is brake vs coast. Brake stops when it stops recieving power, coast will let it coast.
      climberConfig.idleMode(IdleMode.kBrake);
      
      // This assignment gets the encoder from the motor object defined earlier. A RelativeEncoder is an object created with each CANSparkMax controller.
      climberCoder = climberMotor.getEncoder();
      climberController = climberMotor.getClosedLoopController();
  
      // Preferences.initDouble("ClimberP", 0.0006);
      // Preferences.initDouble("ClimberI", 0);
      // Preferences.initDouble("ClimberD", 0);
  
      // climberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // .p(Preferences.getDouble("ClimberP", 0.0006))
      // .i(Preferences.getDouble("ClimberI", 0))
      // .d(Preferences.getDouble("ClimberD", 0))
      // .feedForward.kV(0.00018);
      
      //11111cchbbbbbbbbbbbbbbbbbbbbbbbbbbyyyb333333eeeeeeeeeeeeeeeeeeeee BUNNY CODE!! `
  
      climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  /**
   * Simple function to spin the intake motor at the parameter speed. 
   * @param speed Speed between -1.0 and 1.0.
   */
  public void positionClimber(double position) {
    // set(speed) is the simple way to set speed for a SparkMAX. It differs slightly from a Talon - see another subsystem for that.
    // intakeController.setReference(speed, ControlType.kVelocity);
    climberController.setSetpoint(position, ControlType.kPosition);
  }

  public double getClimberPosition(){
    return climberCoder.getPosition();
  }
  
  /**
   * Simple function to stop the intake. It is good to have this as opposed to running spinIntake(0), because it ensures there is no error.
   */
  public void stopClimber() {
   //intakeController.setReference(0, ControlType.kVelocity);
   climberMotor.set(0);
  }
  
  /**
   * Periodic function standard to all subsystems.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position", getClimberPosition());
  }
}