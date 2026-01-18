
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

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

private final SparkClosedLoopController leftPivotController;
  private final SparkMax spinMotor;
  private final SparkMax leftPivotMotor;
  private final SparkMax rightPivotMotor;

  private final SparkMaxConfig spinConfig;
  private final SparkMaxConfig leftPivotConfig;
  private final SparkMaxConfig rightPivotConfig;

  //private final SparkClosedLoopController intakeController;

  private final RelativeEncoder spinCoder;
  private final RelativeEncoder leftPivotCoder;
  private final RelativeEncoder rightPivotCoder;

  private final DigitalInput intakeBeamBreak;

  public IntakeSubsystem() {


    //DEVICE ID WILL CHANGE WHEn ROBOT.
    spinMotor = new SparkMax(55, MotorType.kBrushless);
    leftPivotMotor = new SparkMax(55, MotorType.kBrushless);
    rightPivotMotor = new SparkMax(55, MotorType.kBrushless);

    spinConfig = new SparkMaxConfig();
    leftPivotConfig = new SparkMaxConfig();
    rightPivotConfig = new SparkMaxConfig();

    //intakeController = spinMotor.getClosedLoopController();
    intakeBeamBreak = new DigitalInput(1);
    
  
    // IdleMode is brake vs coast. Brake stops when it stops recieving power, coast will let it coast.
    spinConfig.idleMode(IdleMode.kBrake);
    leftPivotConfig.idleMode(IdleMode.kBrake);
    rightPivotConfig.idleMode(IdleMode.kBrake);
    rightPivotConfig.follow(leftPivotMotor);

    // This assignment gets the encoder from the motor object defined earlier. A RelativeEncoder is an object created with each CANSparkMax controller.
    spinCoder = spinMotor.getEncoder();
    rightPivotCoder = rightPivotMotor.getEncoder();
    leftPivotCoder = leftPivotMotor.getEncoder();

    leftPivotController = leftPivotMotor.getClosedLoopController();


    spinConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(SmartDashboard.getNumber("IntakeP", 0.001))
    .i(SmartDashboard.getNumber("IntakeI", 0))
    .d(SmartDashboard.getNumber("IntakeD", 0))
    .feedForward.kV(5.0/5767);

    leftPivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(SmartDashboard.getNumber("IntakeP", 0.001))
    .i(SmartDashboard.getNumber("IntakeI", 0))
    .d(SmartDashboard.getNumber("IntakeD", 0))
    .feedForward.kV(5.0/5767);

    rightPivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(SmartDashboard.getNumber("IntakeP", 0.001))
    .i(SmartDashboard.getNumber("IntakeI", 0))
    .d(SmartDashboard.getNumber("IntakeD", 0))
    .feedForward.kV(5.0/5767);
    
    //11111cchbbbbbbbbbbbbbbbbbbbbbbbbbbyyyb333333eeeeeeeeeeeeeeeeeeeee BUNNY CODE!! `

    spinMotor.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightPivotMotor.configure(rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftPivotMotor.configure(leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // A digital input is the slots 0-9 on the RoboRIO in the "DIO" area. You plug in limit switches into here normally. Essentially, this declaration points to the number 9 slot on the DIO. 
  }

    public boolean isIntakeBeamBreakTripped() {
        return intakeBeamBreak.get();
    }

  
  /**
   * Simple function to spin the intake motor at the parameter speed. 
   * @param speed Speed between -1.0 and 1.0.
   */
  public void spinIntake(double speed) {
    // set(speed) is the simple way to set speed for a SparkMAX. It differs slightly from a Talon - see another subsystem for that.
    // intakeController.setReference(speed, ControlType.kVelocity);
    spinMotor.set(speed);
  }
  
  /**
   * Simple function to stop the intake. It is good to have this as opposed to running spinIntake(0), because it ensures there is no error.
   */
  public void stopIntake() {
   //intakeController.setReference(0, ControlType.kVelocity);
   spinMotor.set(0);
  }

  public void intakePivotPosition(double pivotPosition){
  leftPivotController.setSetpoint(pivotPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, .08, ArbFFUnits.kPercentOut);
  }




  // public void Rumble() {
  //   driverJoystick.setRumble
  // }

  /**
   * Returns a boolean on whether the limit switches were tripped in the intake mechanism. Notice how the value is negated - this is a simple code change due to wiring necessities.
   * It is much easier to change code like this than wires.
   * @return Boolean for whether limit is tripped. True is tripped, false is not.
   */
  
  /**
   * Periodic function standard to all subsystems.
   */
  @Override
  public void periodic() {
    i = i+1;
    SmartDashboard.putNumber("Intake Speed", getIntakeSpeed());
     SmartDashboard.putNumber("Intake Encoder", getIntakeEncoder());
      double newP = SmartDashboard.getNumber("IntakeP", 0.001);
      double newI = SmartDashboard.getNumber("IntakeI", 0);
      double newD = SmartDashboard.getNumber("IntakeD", 0);

     if (newP != PreviousP || newI != PreviousI || newD != PreviousD){
    
      spinConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(SmartDashboard.getNumber("IntakeP", 0.001))
      .i(SmartDashboard.getNumber("IntakeI", 0))
      .d(SmartDashboard.getNumber("IntakeD", 0));  

     spinMotor.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
     }
     PreviousP = newP;
     PreviousI = newI;
     PreviousD = newD; 

     SmartDashboard.putBoolean("Top Beam Break Tripped?", istopbeambreaktripped());
     SmartDashboard.putBoolean("Bottom Beam Break Tripped?", isbottombeambreaktripped());
   }
}