// Reapplies the offsets set by the preferences
// without having to deploy code.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorAbsolutePosition extends Command {
  /** Creates a new instance of ElevatorShift. */
  private final ElevatorSubsystem elevatorSubsystem;
  private final double clockOut;

  public ElevatorAbsolutePosition(ElevatorSubsystem elevatorSubsystem, double clockIn) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.clockOut = clockIn;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double target = this.clockOut;
    elevatorSubsystem.setPosition(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}