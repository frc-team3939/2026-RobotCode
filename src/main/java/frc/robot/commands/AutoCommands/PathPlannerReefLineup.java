// Reapplies the offsets set by the preferences
// without having to deploy code.

package frc.robot.commands.AutoCommands;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class PathPlannerReefLineup extends Command {

  PhotonCamera rightCamera = new PhotonCamera("ArducamRight");
  PhotonCamera leftCamera = new PhotonCamera("ArducamLeft");
  boolean rightside = true;
  Set<Integer> validIDs = new HashSet<>(Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));
  private final Transform3d robotToCam;
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonPoseEstimator photonPoseEstimator;
  private Command pathCommand;

  /** Creates a new instance of CameraCoralLineup. */
  private final SwerveSubsystem swerveSubsystem;
    
    public PathPlannerReefLineup(SwerveSubsystem swerveSubsystem, boolean rightside) {
      aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
      robotToCam = new Transform3d(new Translation3d(-0.24, 0.23, 0.27), new Rotation3d(0.0, 0.0, 45/180*Math.PI));
      photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
      this.swerveSubsystem = swerveSubsystem;
      this.rightside = rightside;
      addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   //System.out.println("INIT!**********************************************************");
   //System.out.println("INIT!**********************************************************");
   //System.out.println("INIT!**********************************************************");
   pathCommand = null;
  }

  public Pose2d getTargetLocation(int fiducial){
    if (rightside){
      if (fiducial == 8){
        return new Pose2d(13.542, 5.254, Rotation2d.fromDegrees(150));
      }
    }else{
      if (fiducial == 8){
        return new Pose2d(13.837, 5.035, Rotation2d.fromDegrees(150));
      }
    }
    return new Pose2d(0,0,new Rotation2d(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    var result = leftCamera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      //System.out.println("FOUND A TARGET! **************************");
      var robotPose = photonPoseEstimator.update(result);
      if (pathCommand != null && pathCommand.isScheduled()){
        return;
      }
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget target : targets) {
            if (validIDs.contains(target.getFiducialId())) {
              if (robotPose.isPresent()){
                //System.out.println("FOUND AN APRILTAG");
                //System.out.println("ESTIMATED POSE: ");
                EstimatedRobotPose res = robotPose.get();
                
                Pose2d currentPose = res.estimatedPose.toPose2d();
                currentPose = new Pose2d(currentPose.getX(),currentPose.getY(),new Rotation2d((currentPose.getRotation().getDegrees() - 45)/180*Math.PI));
                //System.out.println(currentPose);

                List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose,
                  getTargetLocation(target.getFiducialId()));

                Pose2d delta = getTargetLocation(target.getFiducialId()).relativeTo(currentPose);
                double distance = Math.sqrt(delta.getX()*delta.getX() + delta.getY()*delta.getY());
                SmartDashboard.putNumber("PathPlannerReefLineup Dist to Go", distance);
                if (distance < 0.05){
                  return;
                }
                swerveSubsystem.resetOdometry(currentPose);
                PathConstraints constraints = new PathConstraints(1, 1, 3*Math.PI, 4*Math.PI);

                PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, getTargetLocation(target.getFiducialId()).getRotation()));

                //System.out.println("PATHPLANNER PATH: ");
                //System.out.println(path);
                
                path.preventFlipping = true;
                pathCommand = new ScheduleCommand(AutoBuilder.followPath(path));
                // pathCommand.schedule();
                CommandScheduler.getInstance().schedule(pathCommand);
                //System.out.println("SCHEDULED COMMAND! ********************");
              }
                
            }
            
        }


    }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("END OF PATHPLANNERREEFLINEUP ***********");
    System.out.println(swerveSubsystem.getPose());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pathCommand != null){
      //System.out.print("isScheduled????? Value: ");
      //System.out.println(pathCommand.isScheduled());
      if (pathCommand.isScheduled()){
        //System.out.print("isFinished????? Value: ");
        //System.out.println(pathCommand.isFinished());
        return pathCommand.isFinished();
      } else {
        return false;
      }
    }
    return false;
  }
}
