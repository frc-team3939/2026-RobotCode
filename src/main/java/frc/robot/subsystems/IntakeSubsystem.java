
package frc.robot.subsystems;

 //import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
 import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
 import com.revrobotics.spark.SparkBase.ResetMode;
 import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final SparkMax intakeMotor;
  private final SparkMaxConfig intakeConfig;

  //private final SparkClosedLoopController intakeController;

  private final RelativeEncoder intakeCoder;

  private int i = 0;

  private double PreviousP;
  private double PreviousI;
  private double PreviousD;

  private final DigitalInput topbeamBreak;
  private final DigitalInput bottombeamBreak;

  public IntakeSubsystem() {


    //Change ID once robot is wired
    intakeMotor = new SparkMax(55, MotorType.kBrushless);
    intakeConfig = new SparkMaxConfig();

    //intakeController = intakeMotor.getClosedLoopController();
    topbeamBreak = new DigitalInput(1);
    bottombeamBreak = new DigitalInput(0);
    
    SmartDashboard.setDefaultNumber("IntakeP", 0.001);
    SmartDashboard.setDefaultNumber("IntakeI", 0);
    SmartDashboard.setDefaultNumber("IntakeD", 0);

    // IdleMode is brake vs coast. Brake stops when it stops recieving power, coast will let it coast.
    intakeConfig.idleMode(IdleMode.kBrake);

    // This assignment gets the encoder from the motor object defined earlier. A RelativeEncoder is an object created with each CANSparkMax controller.
    intakeCoder = intakeMotor.getEncoder();
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(SmartDashboard.getNumber("IntakeP", 0.001))
    .i(SmartDashboard.getNumber("IntakeI", 0))
    .d(SmartDashboard.getNumber("IntakeD", 0))
    .velocityFF(5.0/5767);

    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // A digital input is the slots 0-9 on the RoboRIO in the "DIO" area. You plug in limit switches into here normally. Essentially, this declaration points to the number 9 slot on the DIO. 
  }

    public boolean istopbeambreaktripped() {
        return topbeamBreak.get();
    }

    public boolean isbottombeambreaktripped() {
      return bottombeamBreak.get();
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
  
  /**
   * Simple function to stop the intake. It is good to have this as opposed to running spinIntake(0), because it ensures there is no error.
   */
  public void stopIntake() {
   //intakeController.setReference(0, ControlType.kVelocity);
   intakeMotor.set(0);
  }

  /**
   * Returns the intake encoder position. The spinning wheels do have a position - the encoder is located inside the NEO.
   * @return The double encoder position.
   */
   public double getIntakeEncoder() {
     return intakeCoder.getPosition();
   }

  public double getIntakeSpeed() {
    // return 0;
    return intakeCoder.getVelocity();
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
    
      intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(SmartDashboard.getNumber("IntakeP", 0.001))
      .i(SmartDashboard.getNumber("IntakeI", 0))
      .d(SmartDashboard.getNumber("IntakeD", 0));  

     intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
     }
     PreviousP = newP;
     PreviousI = newI;
     PreviousD = newD; 

     SmartDashboard.putBoolean("Top Beam Break Tripped?", istopbeambreaktripped());
     SmartDashboard.putBoolean("Bottom Beam Break Tripped?", isbottombeambreaktripped());
   }
}