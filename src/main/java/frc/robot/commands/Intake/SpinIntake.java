package frc.robot.commands.Intake;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class SpinIntake extends Command {
  private final IntakeSubsystem intakeSubsystem;
  
  private double speed;
  public SpinIntake(IntakeSubsystem isubsystem, double speed) {
    intakeSubsystem = isubsystem;
    this.speed = speed;
    addRequirements(intakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.spinIntake(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();

    
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