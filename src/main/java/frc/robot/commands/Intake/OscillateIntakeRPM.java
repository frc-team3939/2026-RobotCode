package frc.robot.commands.Intake;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class OscillateIntakeRPM extends Command {
  private IntakeSubsystem intakeSubsystem;
  private int counter;
  private boolean flip;
    
  private double speed;
  public OscillateIntakeRPM(IntakeSubsystem isubsystem, double speed) {
    intakeSubsystem = isubsystem;
    this.speed = speed;
    addRequirements(intakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    flip = true;
    intakeSubsystem.spinIntakeRPM(speed);
    intakeSubsystem.lowAmps();
  }

  // Called every time the scheduler runs while the command is scheduled every 20ms.
  @Override
  public void execute() {
    if (counter <= 50) {
      intakeSubsystem.spinIntakeRPM(-speed);
    }
    else if (counter <= 54) {
     intakeSubsystem.spinIntake(speed/16);
    //  intakeSubsystem.stopIntake();
    }
    else {
      counter = 0;
    }
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
    intakeSubsystem.highAmps();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // if (intakeSubsystem.isbeambreaktripped()) {
    //return true; 
   //} 
    //else{
      return false;
    //}  

  }
}