// Reapplies the offsets set by the preferences
// without having to deploy code.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;


import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class FancyReeflLineup extends Command {

  PhotonCamera rightCamera = new PhotonCamera("ArduCam1");
  PhotonCamera leftCamera = new PhotonCamera("ArduCam2");
  boolean rightside = true;
  Set<Integer> validIDs = new HashSet<>(Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));
  private PIDController xSpdController, ySpdController, turningSpdController;
  private double xSpeed, ySpeed, turningSpeed;
  private SlewRateLimiter xLimiter, yLimiter, turningLimiter;


  /** Creates a new instance of CameraCoralLineup. */
  private final SwerveSubsystem swerveSubsystem;
    
    public FancyReeflLineup(SwerveSubsystem swerveSubsystem, boolean rightside) {
      this.swerveSubsystem = swerveSubsystem;
      this.rightside = rightside;
      xSpdController = new PIDController(0.011, 0, 0);
      xSpdController.setTolerance(2);
      ySpdController = new PIDController(0.013, 0, 0);
      ySpdController.setTolerance(2.5);
      turningSpdController = new PIDController(0.01, 0, 0);
      turningSpdController.setTolerance(2.5);
      this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
      addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform3d cameraToRobot = new Transform3d(0,0,0, new Rotation3d(0, 0, 45/180*Math.PI));
    var result = rightCamera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget target : targets) {
            if (validIDs.contains(target.getFiducialId())) {
              Pose3d tagPose = new Pose3d(target.bestCameraToTarget.toMatrix());
              //target.bestCameraToTarget;
              Pose3d rotatedTarget = tagPose.transformBy(cameraToRobot);
                
            }
            
        }


    }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
