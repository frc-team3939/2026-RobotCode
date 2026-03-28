
// Resets the current rotation of the gyro
// to be considered 0.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DistanceTest extends Command {
  /** Creates a new ZeroHeading. */
  private final SwerveSubsystem swerveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  public DistanceTest(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(swerveSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Transform2d offset = this.swerveSubsystem.getPose().minus(new Pose2d(4.612,4.021,Rotation2d.kZero));
    double distance = Math.sqrt(offset.getX()*offset.getX()+offset.getY()*offset.getY());
    shooterSubsystem.logDistance(distance,distance);
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