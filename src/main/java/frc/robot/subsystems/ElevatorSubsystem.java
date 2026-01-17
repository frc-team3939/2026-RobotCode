
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final SparkMax elevatorMotorOne;
  private final SparkMax elevatorMotorTwo;

  private final SparkMaxConfig elevatorConfigOne;
  private final SparkMaxConfig elevatorConfigTwo;

  private final RelativeEncoder elevatorCoderOne;
  private final RelativeEncoder elevatorCoderTwo;

 // private final DigitalInput bottomLimitSwitch;

  private final SparkClosedLoopController elevatorController;

  // private double PreviousP;
  // private double PreviousI;
  // private double PreviousD;

  private double liftPosition;


  public ElevatorSubsystem() {

    Preferences.initDouble("Intake P", 0.0002);

    //Change ID once robot is wired
    elevatorMotorOne = new SparkMax(28, MotorType.kBrushless);
    elevatorMotorTwo = new SparkMax(51, MotorType.kBrushless);

    elevatorConfigOne = new SparkMaxConfig();
    elevatorConfigTwo = new SparkMaxConfig();

    elevatorController = elevatorMotorOne.getClosedLoopController();

    liftPosition = 0;

    SmartDashboard.setDefaultNumber("ElevatorP", 0.1);
    SmartDashboard.setDefaultNumber("ElevatorI", 0.0002);
    SmartDashboard.setDefaultNumber("ElevatorD", 0);




     //IdleMode is brake vs coast. Brake stops when it stops recieving power, coast will let it coast.
    elevatorConfigOne.idleMode(IdleMode.kBrake);
    elevatorConfigTwo.idleMode(IdleMode.kBrake);

    // This assignment gets the encoder from the motor object defined earlier. A RelativeEncoder is an object created with each CANSparkMax controller.
    elevatorCoderOne = elevatorMotorOne.getEncoder();
    elevatorCoderTwo = elevatorMotorTwo.getEncoder();

    elevatorConfigTwo.follow(elevatorMotorOne);
    // Pulley has 18 teeth, belt has 5mm pitch, gearbox ratio is 4.67:1
    // 18*5mm/pi/2/4.67 = 0.1208inches 
    elevatorConfigOne.encoder.positionConversionFactor(0.1208*4.67*4.375/4);

    // unit is inches per min.
    elevatorConfigOne.encoder.velocityConversionFactor(0.1208*4.67*4.375/4);
    
    elevatorConfigOne.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1)
    .i(0)
    .d(0)
    .outputRange(-0.35, 0.35).iMaxAccum(10);

    // elevatorConfigOne.closedLoop.maxMotion
    // .maxVelocity(1280)
    // .maxAcceleration(12800)
    // .allowedClosedLoopError(0);

    resetPosition();

    elevatorMotorOne.configure(elevatorConfigOne, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotorTwo.configure(elevatorConfigTwo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void setPosition(double target) {
  
    liftPosition = target;
  }
  
  @Override
  public void periodic() {
    if (liftPosition < 1) {
     elevatorMotorOne.set(0);
     elevatorMotorTwo.set(0);
    }
    else {
      //elevatorController.setReference(liftPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, .08, ArbFFUnits.kPercentOut);
      elevatorController.setReference(liftPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, .08, ArbFFUnits.kPercentOut);
    }

   // elevatorMotorOne.set(0);
    //elevatorMotorTwo.set(0);
    SmartDashboard.putNumber("elevator position",this.elevatorCoderOne.getPosition());
    SmartDashboard.putNumber("elevator target",liftPosition);
    SmartDashboard.putNumber("Favorite Number", 7);

  //   double newP = SmartDashboard.getNumber("ElevatorP", 0.1);
  //   double newI = SmartDashboard.getNumber("ElevatorI", 0.0002);
  //   double newD = SmartDashboard.getNumber("ElevatorD", 0);

  //  if (newP != PreviousP || newI != PreviousI || newD != PreviousD){
  
  //   elevatorConfigOne.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  //   .p(SmartDashboard.getNumber("ElevatorP", 0.1))
  //   .i(SmartDashboard.getNumber("ElevatorI", 0.0002))
  //   .d(SmartDashboard.getNumber("ElevatorD", 0));  

  //  elevatorMotorOne.configure(elevatorConfigOne, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  //  }
  //  PreviousP = newP;
  //  PreviousI = newI;
  //  PreviousD = newD; 
  }

  public double getPosition() {
    return liftPosition; 
  }

  public double getEncoder() {
    return elevatorCoderOne.getPosition();
  }

  public void resetPosition() {
    elevatorCoderOne.setPosition(0);
    elevatorCoderTwo.setPosition(0);
    liftPosition = 0;
  }

}