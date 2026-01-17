package frc.robot.commands.AutoCommands;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class BasicReefLineup extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<PhotonPipelineResult> visionInfo;
    private final PIDController xSpdController, ySpdController, turningSpdController;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private double xSpeed, ySpeed, turningSpeed;
    private PhotonPipelineResult visionResult;
    private int targetLostCounter;
    private Set<Integer> validIDs = new HashSet<>(Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));
    private String side;
    
        /**
         * This command writes swerveModuleStates to a photonvision target.
         * @param swerveSubsystem Requirement parameter
         * @param visionInfo PhotonPipelineResult class, input the result
         * @param finishOnTargetLoss IF true, command stops on target loss if LOST and NOT regained 
         * for 20 scheduler cycles. (400 ms) For every cycle there is a target, the counter loses 1.
         */
        public BasicReefLineup(SwerveSubsystem swerveSubsystem, Supplier<PhotonPipelineResult> visionInfo, String side, boolean finishOnTargetLoss) {
            this.side = side;
            this.swerveSubsystem = swerveSubsystem;
            this.visionInfo = visionInfo;
            xSpdController = new PIDController(0.011, 0, 0);
            xSpdController.setTolerance(2.5);
            ySpdController = new PIDController(0.026, 0, 0);
            ySpdController.setTolerance(2.5);
            turningSpdController = new PIDController(0.01, 0, 0);
            turningSpdController.setTolerance(2.5);
            this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
            this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
            this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

            targetLostCounter = 0;
            addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("getY", 5);
    }

    @Override
    public void execute() {
        // get vision info
        visionResult = visionInfo.get();
        if (visionResult.hasTargets() == true) {
                PhotonTrackedTarget target = visionResult.getBestTarget();
                if (validIDs.contains(target.getFiducialId()) && (side.equals("left")|| side.equals("right"))) {
                    if (side.equals("left")) {
                        xSpeed = -xSpdController.calculate(target.getPitch(), 6.2);
                        ySpeed = -ySpdController.calculate(target.getYaw(), 23.11);
                        // turningSpeed = -turningSpdController.calculate(target.getSkew(), 0);
                    }
                    else if (side.equals("right")){
                        xSpeed = -xSpdController.calculate(target.getPitch(), 0);
                        ySpeed = -ySpdController.calculate(target.getYaw(), 0);
                        // turningSpeed = -turningSpdController.calculate(target.getSkew(), 0);
                    }
                    targetLostCounter = targetLostCounter > 0 ? (targetLostCounter - 1) : 0;
                    SmartDashboard.putNumber("getY", target.getYaw());
                }
                else {
                    xSpeed = 0;
                    ySpeed = 0;
                    turningSpeed = 0;
                    targetLostCounter++;
                }  
        } 
        else {
            // if no target, all speeds are ZERO.
            //xSpdController.calculate(0, 21.7);
           // ySpdController.calculate(0, 3);
           // xSpdController.calculate(0, 19.5);
            //ySpdController.calculate(0, 9);
            xSpeed = 0;
            ySpeed = 0;
            turningSpeed = 0;
            targetLostCounter++;
        }
        turningSpeed = 0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 2;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 2;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 2;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.3, 0);
        SmartDashboard.putNumber("visionxspeed", xSpeed);
        SmartDashboard.putNumber("visionyspeed", ySpeed);
        SmartDashboard.putNumber("visionturningspeed", turningSpeed);
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() { 
        // If finishontargetloss is true, the command will exit if it loses its target. 
        // Good for distinction between autonomous and teleoperated control. 
        // We would want it to complete if it loses target in teleop, but not in autonomous such that it does not complete the rest of the sequence.
        // return finishOnTargetLoss == true ?
        if((xSpdController.atSetpoint() && ySpdController.atSetpoint()) || (targetLostCounter >= 25)){
            return true;
        }
        return false;
    }
}