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
    private AddressableLED leds = new AddressableLED(9);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(90);
    private double error;

    PhotonCamera elevatorCamera = new PhotonCamera("ElevatorCam");
    
    public LEDSubsystem() { 
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
        LEDPattern targetcolor = LEDPattern.solid(new Color (0,255,0));
        targetcolor.applyTo(buffer);
        leds.setData(buffer);
    }

    @Override
    public void periodic(){
        LEDPattern targetcolor = LEDPattern.solid(new Color (0,255,0));
        targetcolor.applyTo(buffer);
        leds.setData(buffer);
        
    }
}
