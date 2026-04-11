package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRPM extends Command{

    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final double speed;
    private final double speed_feeder;

    private int i;

    public ShooterRPM(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, double speed, double speed_feeder_rpm){
        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.speed = speed;
        this.speed_feeder = speed_feeder_rpm;
        this.i = 0;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //swerveSubsystem.resetHeading();
        this.i = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.shooterSubsystem.spinShooterSpeed(speed);
        // this.feederSubsystem.feederSpin(this.speed_feeder);
        // this.feederSubsystem.spinBelt(this.speed_feeder);
        // double shooter_speed_error = this.shooterSubsystem.getShooterSpeed() - Preferences.getDouble("Shooter RPM",0.0);
        if (this.i > 100) {
            this.feederSubsystem.spinFeederSpeed(this.speed_feeder);
            this.feederSubsystem.spinBeltSpeed(this.speed_feeder);
        } else {
            // this.feederSubsystem.spinFeederSpeed(0);
            // this.feederSubsystem.spinBeltSpeed(0);
        }
        this.i += 1;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.stopShooter();
        this.feederSubsystem.feederSpin(0);
        this.feederSubsystem.stopBelt();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
