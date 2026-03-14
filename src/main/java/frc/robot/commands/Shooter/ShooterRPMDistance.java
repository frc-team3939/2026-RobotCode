package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShooterRPMDistance extends Command{

    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final double speed_feeder;

    private Command driveToPose;
    
    enum State {
        START,
        ROTATE,
        SPIN_SHOOTER,
        SHOOT,
    }
    private int counter;
    private State state;

    public ShooterRPMDistance(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, SwerveSubsystem swerveSubsystem, double speed_feeder){
        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.state = State.START;
       
        this.speed_feeder = speed_feeder;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //swerveSubsystem.resetHeading();
        this.state = State.START;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d hubLocation;
        if (DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue){
            hubLocation = new Pose2d(4.612,4.021,Rotation2d.kZero);
        }
        else {
            hubLocation = new Pose2d(11.920,4.021,Rotation2d.kZero);
        }
        Transform2d offset = this.swerveSubsystem.getPose().minus(hubLocation);
        Rotation2d facing_angle = new Rotation2d(offset.getX(),offset.getY());
        double distance = Math.sqrt(offset.getX()*offset.getX()+offset.getY()*offset.getY());
        this.shooterSubsystem.logDistance(distance);
        switch (this.state){
            case START:
            this.state = State.ROTATE;
            Pose2d targetPose = new Pose2d(this.swerveSubsystem.getPose().getTranslation(),facing_angle);
            // this.driveToPose = this.swerveSubsystem.driveToPose(targetPose);
            // CommandScheduler.getInstance().schedule(this.driveToPose);
            break;
            case ROTATE:
            // if (this.driveToPose.isFinished()){
            //     this.state = State.SPIN_SHOOTER;
                 counter = 0;
            // }
            this.state = State.SPIN_SHOOTER;
            break;
            case SPIN_SHOOTER:
            counter = counter + 1;
           // Transform2d offset = this.swerveSubsystem.getPose().minus(new Pose2d(4.612,4.021,Rotation2d.kZero));
           //double distance = Math.sqrt(offset.getX()*offset.getX()+offset.getY()*offset.getY());
            //if (Math.abs(shooterSubsystem.getShooterSpeed()-shooterSubsystem.getRPMFromDistance(distance))<= 100){
            if (counter >= 75){
                this.state = State.SHOOT;
            }
            break;
            case SHOOT:
            break;

        }

        switch (this.state){
            case START:
            break;
            case ROTATE:
            break;
            case SPIN_SHOOTER:
            this.shooterSubsystem.spinShooterSpeed(this.shooterSubsystem.getRPMFromDistance(distance));
            break;
            case SHOOT:
            this.shooterSubsystem.spinShooterSpeed(this.shooterSubsystem.getRPMFromDistance(distance));
            this.feederSubsystem.feederSpin(this.speed_feeder);
            this.feederSubsystem.spinBelt(this.speed_feeder);
            break;
        }
        //this.shooterSubsystem.spinShooterSpeed(speed);
        // this.feederSubsystem.feederSpin(this.speed_feeder);
        // this.feederSubsystem.spinBelt(this.speed_feeder);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.stopShooter();
        this.feederSubsystem.feederSpin(0);
        this.feederSubsystem.stopBelt();
        //this.shooterSubsystem.spinShooterSpeed();
       // this.feederSubsystem.feederSpin(0);
        //this.feederSubsystem.stopBelt();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
