package frc.robot.commands.Intake;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class IntelligentIntake extends Command {
  private final IntakeSubsystem intakeSubsystem;
  
  enum State {
    FULL_SPEED,
    HALF_SPEED,
    END,
  }

  private State state;
  

  private double speed;
  public IntelligentIntake(IntakeSubsystem isubsystem, double speed) {
    intakeSubsystem = isubsystem;
    this.speed = speed;
    addRequirements(intakeSubsystem);
    state = State.FULL_SPEED;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = State.FULL_SPEED;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (state) {
      case FULL_SPEED:
      // if (intakeSubsystem.istopbeambreaktripped() ==false) {
      //   state = State.HALF_SPEED;
      // }
      break;

      case HALF_SPEED:
      // if (intakeSubsystem.isbottombeambreaktripped() ==false) {
      //   state = State.END;
      // }
      break;

      case END:
      break;
    }
    switch (state) {
      case FULL_SPEED:
      intakeSubsystem.spinIntake(speed);
      break;

      case HALF_SPEED:
      intakeSubsystem.spinIntake(speed/2);
      break;

      case END:
      intakeSubsystem.stopIntake();
      break;
    }
    SmartDashboard.putString("Intelligent Intake State", state.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (state ==State.END) {
    return true; 
   } 
    else{
      return false;
    }  

  }
}