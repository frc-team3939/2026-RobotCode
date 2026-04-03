package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkClosedLoopController feederController;
  private final SparkClosedLoopController beltController;

  private final SparkMax beltMotor;
  private final SparkMax feederMotor;
 

  private final SparkMaxConfig beltConfig;
  private final SparkMaxConfig feederConfig;
  

  //private final SparkClosedLoopController intakeController;

  private final RelativeEncoder beltCoder;
  private final RelativeEncoder feederCoder;
  

  public FeederSubsystem() {


    //DEVICE ID WILL CHANGE WHEn ROBOT.
    beltMotor = new SparkMax(3, MotorType.kBrushless);
    feederMotor = new SparkMax(7, MotorType.kBrushless);
    

    beltConfig = new SparkMaxConfig();
    feederConfig = new SparkMaxConfig();
    
  
    // IdleMode is brake vs coast. Brake stops when it stops recieving power, coast will let it coast.
    beltConfig.idleMode(IdleMode.kBrake);
    feederConfig.idleMode(IdleMode.kBrake);

    feederConfig.inverted(true);
  
    feederConfig.smartCurrentLimit(40);
    beltConfig.smartCurrentLimit(40);


    // This assignment gets the encoder from the motor object defined earlier. A RelativeEncoder is an object created with each CANSparkMax controller.
    beltCoder = beltMotor.getEncoder();
    feederCoder = feederMotor.getEncoder();
    

    feederController = feederMotor.getClosedLoopController();
    beltController = beltMotor.getClosedLoopController();


    Preferences.initDouble("BeltP", 0.0006);
    Preferences.initDouble("BeltI", 0.0);
    Preferences.initDouble("BeltD", 0.0);

    Preferences.initDouble("FeederP", 0.0006);
    Preferences.initDouble("FeederI", 0.0);
    Preferences.initDouble("FeederD", 0.0);

    beltConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(Preferences.getDouble("BeltP", 0.0006))
    .i(Preferences.getDouble("BeltI", 0))
    .d(Preferences.getDouble("BeltD", 0))
    .feedForward.kV(0.00018);

    feederConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(Preferences.getDouble("FeederP", 0.0006))
    .i(Preferences.getDouble("FeederI", 0))
    .d(Preferences.getDouble("FeederD", 0))
    .feedForward.kV(0.00018);

    feederConfig.encoder.uvwAverageDepth(16);
    feederConfig.encoder.uvwMeasurementPeriod(20);
    beltConfig.encoder.uvwAverageDepth(16);
    beltConfig.encoder.uvwMeasurementPeriod(20);


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
  
  public void spinBeltSpeed(double rpm) {
    beltController.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void spinFeederSpeed(double rpm) {
    feederController.setSetpoint(rpm, ControlType.kVelocity);
  }

  /**
   * Simple function to stop the intake. It is good to have this as opposed to running spinIntake(0), because it ensures there is no error.
   */
  public void stopBelt() {
   //intakeController.setReference(0, ControlType.kVelocity);
   beltMotor.set(0);
  }

  public void feederPosition(double rotatePosition){
  feederController.setSetpoint(rotatePosition, ControlType.kVelocity, ClosedLoopSlot.kSlot0, .08, ArbFFUnits.kPercentOut);
  }

  public void feederSpin(double speed){
    feederMotor.set(speed);
  }

  public double getFeederVelocity() {
    return feederCoder.getVelocity();
  }

  public double getBeltVelocity() {
    return beltCoder.getVelocity();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder RPM", getFeederVelocity());
    SmartDashboard.putNumber("Belt RPM", getBeltVelocity());
    
    // if (this.i % 50 == 0){
    //   beltConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // .p(Preferences.getDouble("BeltP", 0.0006))
    // .i(Preferences.getDouble("BeltI", 0))
    // .d(Preferences.getDouble("BeltD", 0))
    // .feedForward.kV(0.00018);

    // feederConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // .p(Preferences.getDouble("FeederP", 0.0006))
    // .i(Preferences.getDouble("FeederI", 0))
    // .d(Preferences.getDouble("FeederD", 0))
    // .feedForward.kV(0.00018);

    


    // beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // }
   }
}
