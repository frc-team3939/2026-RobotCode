package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleShooter extends Command{

    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final double speed;
    private final double speed_feeder;
    private int counter;

    public SimpleShooter(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, double speed, double speed_feeder){
        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.speed = speed;
        this.speed_feeder = speed_feeder;
        counter = 0;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //swerveSubsystem.resetHeading();
        counter = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.shooterSubsystem.spinShooterSpeed(5767*speed);
        // double speedRatio = this.shooterSubsystem.getShooterSpeed() / speed;
        counter = counter + 1;
        if (counter > 80){
            this.feederSubsystem.feederSpin(this.speed_feeder);
            this.feederSubsystem.spinBelt(this.speed_feeder);
        } else {
            this.feederSubsystem.feederSpin(0);
            this.feederSubsystem.stopBelt();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.spinShooterSpeed(0);
        this.feederSubsystem.feederSpin(0);
        this.feederSubsystem.stopBelt();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
