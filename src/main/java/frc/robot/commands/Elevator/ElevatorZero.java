
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorZero extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  
  enum State {
    POWERED,
    UNPOWERED,
    END,
  }

  private State state;
  

  public ElevatorZero(ElevatorSubsystem esubsystem, double speed) {
    elevatorSubsystem = esubsystem;
    addRequirements(elevatorSubsystem);
    state = State.POWERED;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = State.POWERED;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (state) {
      case POWERED:
      if (elevatorSubsystem.getPosition() <= 1) {
        state = State.UNPOWERED;
      }
      break;

      case UNPOWERED:
      if (true) {
        state = State.END;
      }
      break;

      case END:
      break;
    }
    switch (state) {
      case POWERED:
      elevatorSubsystem.setPosition(1);
      break;

      case UNPOWERED:
      elevatorSubsystem.setPosition(0);
      break;

      case END:
      break;
    }
    SmartDashboard.putString("Elevator State", state.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 elevatorSubsystem.setPosition(0);
    
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