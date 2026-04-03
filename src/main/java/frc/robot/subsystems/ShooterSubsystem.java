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
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

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

  private final DigitalInput shooterBeamBreak;
  private final InterpolatingTreeMap<Double, Double> RPMmap;

  private double rawDistance;
  private double distance;

  public ShooterSubsystem() {

    // DEVICE ID WILL CHANGE WHEn ROBOT.
    leftFlywheelMotor = new SparkMax(8, MotorType.kBrushless);
    rightFlywheelMotor = new SparkMax(9, MotorType.kBrushless);

    leftFlywheelConfig = new SparkMaxConfig();
    rightFlywheelConfig = new SparkMaxConfig();

    // intakeController = spinMotor.getClosedLoopController();
    shooterBeamBreak = new DigitalInput(2);
    
    RPMmap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    // IdleMode is brake vs coast. Brake stops when it stops recieving power, coast
    // will let it coast.
    leftFlywheelConfig.idleMode(IdleMode.kCoast);
    rightFlywheelConfig.idleMode(IdleMode.kCoast);
    rightFlywheelConfig.follow(leftFlywheelMotor, true);

    // leftFlywheelConfig.inverted(true);

    leftFlywheelConfig.smartCurrentLimit(40);
    rightFlywheelConfig.smartCurrentLimit(40);

    // This assignment gets the encoder from the motor object defined earlier. A
    // RelativeEncoder is an object created with each CANSparkMax controller.

    leftFlywheelCoder = leftFlywheelMotor.getEncoder();
    Preferences.initDouble("ShooterP", 0.0004);
    Preferences.initDouble("ShooterI", 0.000001);
    Preferences.initDouble("ShooterD", 0);
    leftFlywheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Preferences.getDouble("ShooterP", 0.0004))
        .i(Preferences.getDouble("ShooterI", 0))
        .d(Preferences.getDouble("ShooterD", 0))
        .feedForward.kV(0.00018);

    flywheelController = leftFlywheelMotor.getClosedLoopController();
    
    leftFlywheelConfig.encoder.uvwAverageDepth(16);
    leftFlywheelConfig.encoder.uvwMeasurementPeriod(20);

    rightFlywheelMotor.configure(rightFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFlywheelMotor.configure(leftFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // A digital input is the slots 0-9 on the RoboRIO in the "DIO" area. You plug
    // in limit switches into here normally. Essentially, this declaration points to
    // the number 9 slot on the DIO.
    RPMmap.put(2.071, -3000.0);
    //RPMmap.put(2.081, -2900.0);
    RPMmap.put(2.642, -3600.0); 
    RPMmap.put(3.299, -3750.0);
    RPMmap.put(4.013, -4575.0);
    RPMmap.put(5.122, -5250.0);
   
    Preferences.initDouble("Shooter RPM", 0.000);
    Preferences.initDouble("ShooterMultiply", 1.0);
    //Preferences.getDouble("Shooter RPM", 0.0);
  }

  public boolean isIntakeBeamBreakTripped() {
    return shooterBeamBreak.get();
  }

  public double getRPMFromDistance(double distance){
    return RPMmap.get(distance) * Preferences.getDouble("ShooterMultiply", 1.0);
  }


  public void spinShooterSpeed(double rpm){
    flywheelController.setSetpoint(rpm, ControlType.kVelocity);
  }
  
  public void spinShooterSpeed(){
    // leftFlywheelMotor.set(Preferences.getDouble("Shooter RPM", 0.0));
    spinShooterSpeed(Preferences.getDouble("Shooter RPM", 0.0));
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
  public void logDistance(double rawDistance, double distance) {
    this.rawDistance = rawDistance;
    this.distance = distance;
  }

  public double getRecentDistance() {
    return this.distance;
  }
  // ynmtocc4 MOrE BUNNY CODE!!!!!
  public boolean isAtSetpoint () {
    return flywheelController.isAtSetpoint();
  }

  public void spinShooter(double power) {
    leftFlywheelMotor.set(power);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterRPM", getShooterSpeed());
    SmartDashboard.putNumber("Recent Calculated Distance (raw)", this.rawDistance*39.3);
    SmartDashboard.putNumber("Recent Calculated Distance", getRecentDistance()*39.3);
  }
}