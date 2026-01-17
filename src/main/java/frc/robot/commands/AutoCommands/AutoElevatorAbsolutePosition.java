// Reapplies the offsets set by the preferences
// without having to deploy code.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoElevatorAbsolutePosition extends Command {
  /** Creates a new instance of ElevatorShift. */
  private final ElevatorSubsystem elevatorSubsystem;
  private final double clockOut;
  private int timeOut;
  private double target;

  public AutoElevatorAbsolutePosition(ElevatorSubsystem elevatorSubsystem, double clockIn) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.clockOut = clockIn;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = this.clockOut;
    elevatorSubsystem.setPosition(target);
    timeOut = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timeOut++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(elevatorSubsystem.getEncoder() - target) <= 0.75 || timeOut >= 150) {
      return true;
    } 
    return false;
  }
}