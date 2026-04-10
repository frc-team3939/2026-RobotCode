
// Resets the current rotation of the gyro
// to be considered 0.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;


public class SpinBelt extends Command {
  /** Creates a new ZeroHeading. */
  private final FeederSubsystem feederSubsystem;
  private final double speed;
  public SpinBelt(FeederSubsystem feederSubsystem, double speed) {
    this.feederSubsystem = feederSubsystem;
    this.speed = speed;
    addRequirements(feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feederSubsystem.spinBeltSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederSubsystem.spinBeltSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stopBelt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}