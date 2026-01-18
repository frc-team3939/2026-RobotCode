package frc.robot.subsystems;

 //import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
private final SparkClosedLoopController feederController;

  private final SparkMax beltMotor;
  private final SparkMax feederMotor;
 

  private final SparkMaxConfig beltConfig;
  private final SparkMaxConfig feederConfig;
  

  //private final SparkClosedLoopController intakeController;

  private final RelativeEncoder beltCoder;
  private final RelativeEncoder feederCoder;
 

  public FeederSubsystem() {


    //DEVICE ID WILL CHANGE WHEn ROBOT.
    beltMotor = new SparkMax(55, MotorType.kBrushless);
    feederMotor = new SparkMax(55, MotorType.kBrushless);
    

    beltConfig = new SparkMaxConfig();
    feederConfig = new SparkMaxConfig();
    
  
    // IdleMode is brake vs coast. Brake stops when it stops recieving power, coast will let it coast.
    beltConfig.idleMode(IdleMode.kBrake);
    feederConfig.idleMode(IdleMode.kBrake);
  

    // This assignment gets the encoder from the motor object defined earlier. A RelativeEncoder is an object created with each CANSparkMax controller.
    beltCoder = beltMotor.getEncoder();
    feederCoder = feederMotor.getEncoder();
    

    feederController = feederMotor.getClosedLoopController();


    beltConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(SmartDashboard.getNumber("IntakeP", 0.001))
    .i(SmartDashboard.getNumber("IntakeI", 0))
    .d(SmartDashboard.getNumber("IntakeD", 0))
    .feedForward.kV(5.0/5767);

    feederConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(SmartDashboard.getNumber("IntakeP", 0.001))
    .i(SmartDashboard.getNumber("IntakeI", 0))
    .d(SmartDashboard.getNumber("IntakeD", 0))
    .feedForward.kV(5.0/5767);

    


    beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // A digital input is the slots 0-9 on the RoboRIO in the "DIO" area. You plug in limit switches into here normally. Essentially, this declaration points to the number 9 slot on the DIO. 
  }

  
  /**
   * Simple function to spin the intake motor at the parameter speed. 
   * @param speed Speed between -1.0 and 1.0.
   */
  public void spinBelt(double speed) {
    // set(speed) is the simple way to set speed for a SparkMAX. It differs slightly from a Talon - see another subsystem for that.
    // intakeController.setReference(speed, ControlType.kVelocity);
    beltMotor.set(speed);
  }
  
  /**
   * Simple function to stop the intake. It is good to have this as opposed to running spinIntake(0), because it ensures there is no error.
   */
  public void stopBelt() {
   //intakeController.setReference(0, ControlType.kVelocity);
   beltMotor.set(0);
  }

  public void feederPosition(double rotatePosition){
  feederController.setSetpoint(rotatePosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, .08, ArbFFUnits.kPercentOut);
  }
  
  @Override
  public void periodic() {
    
   }
}
