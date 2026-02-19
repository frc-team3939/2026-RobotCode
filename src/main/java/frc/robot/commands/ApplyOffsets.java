
// Reapplies the offsets set by the preferences
// without having to deploy code.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OldSwerveSubsystem;

public class ApplyOffsets extends Command {
  /** Creates a new instance of ApplyOffsets. */
  private final OldSwerveSubsystem swerveSubsystem;
    public ApplyOffsets(OldSwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.applyOffsets();
    
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
