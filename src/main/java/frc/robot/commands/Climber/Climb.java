package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class Climb extends Command {
  /** Creates a new instance of Climb. */
    private final ClimberSubsystem climberSubsystem;
    private double position;
    public Climb(ClimberSubsystem climberSubsystem, double position) {
        this.climberSubsystem = climberSubsystem;
        this.position = position;
        addRequirements(climberSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberSubsystem.positionClimber(position);
    
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

