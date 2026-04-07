package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED leds = new AddressableLED(9);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(90);
    private double error;

// //     PhotonCamera elevatorCamera = new PhotonCamera("ElevatorCam");

// //     {
// //   Optional<Alliance> alliance = DriverStation.getAlliance();
// //   // If we have no alliance, we cannot be enabled, therefore no hub.
// //   if (alliance.isEmpty()) {
    
// //   }
// //   // Hub is always enabled in autonomous.
// //   if (DriverStation.isAutonomousEnabled()) {
    
// //   }
// //   // At this point, if we're not teleop enabled, there is no hub.
// //   if (!DriverStation.isTeleopEnabled()) {
    
// //   }

// //   // We're teleop enabled, compute.
// //   double matchTime = DriverStation.getMatchTime();
// //   String gameData = DriverStation.getGameSpecificMessage();
// //   // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
// //   if (gameData.isEmpty()) {
    
// //   }
// //   boolean redInactiveFirst = false;
// //   switch (gameData.charAt(0)) {
// //     case 'R' -> redInactiveFirst = true;
// //     case 'B' -> redInactiveFirst = false;
// //     default -> {
// //       // If we have invalid game data, assume hub is active.
      
// //     }
// //   }

// //   // Shift was is active for blue if red won auto, or red if blue won auto.
// //   boolean shift1Active = switch (alliance.get()) {
// //     case Red -> !redInactiveFirst;
// //     case Blue -> redInactiveFirst;
// //   };

// //   if (matchTime > 130) {
// //     // Transition shift, hub is active.
   
// //   } else if (matchTime > 105) {
// //     // Shift 1
    
// //   } else if (matchTime > 80) {
// //     // Shift 2
    
// //   } else if (matchTime > 55) {
// //     // Shift 3
    
// //   } else if (matchTime > 30) {
// //     // Shift 4
    
// //   } else {
// //     // End game, hub always active.
    
// //   }
// }
    
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
