package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED leds = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(90);
    private ElevatorSubsystem elevatorSubsystem;
    private double error;

    PhotonCamera elevatorCamera = new PhotonCamera("ElevatorCam");
    
    public LEDSubsystem(ElevatorSubsystem es) { 
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
        elevatorSubsystem = es;
    }

    public boolean colorAligned() {
        return error < 5;
    }

    @Override
    public void periodic(){
        var result = elevatorCamera.getLatestResult();
        error = 3939;
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            // double errorLeft = Math.abs(target.getYaw() - 5);
            // double errorRight = Math.abs(target.getYaw() - -5);
            error = Math.abs(target.getYaw());//Math.min(errorLeft, errorRight);
            SmartDashboard.putNumber("Vision Coral Error", error);
        }
        LEDPattern targetColor;
        if (error < 5){
            targetColor = LEDPattern.solid(new Color(150, 0, 150));
        }
        else if (elevatorSubsystem.getEncoder() < .25) {
                targetColor = LEDPattern.solid(new Color(0,255,0));
        }

        else {
            targetColor = LEDPattern.solid(new Color(255,0,0));
        }

        targetColor.applyTo(buffer);
        leds.setData(buffer);
        SmartDashboard.putNumber("Test", 7);
    }
}
