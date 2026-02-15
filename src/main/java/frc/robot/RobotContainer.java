package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;
import frc.robot.commands.*;
import frc.robot.commands.Elevator.ElevatorAbsolutePosition;
import frc.robot.commands.Elevator.ElevatorZero;
import frc.robot.commands.Intake.IntelligentIntake;
import frc.robot.commands.Intake.SpinIntake;
import frc.robot.commands.AutoCommands.*;

import java.io.File;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
 
    /*
    "Imports" subsystems that you make in the subsystem folder to be used for controller actions.
    Make sure you actually import the subsystem in the same manner as we do with the SwerveSubsystem above.
    */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();


    // private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    // private final LEDSubsystem ledSubsystem = new LEDSubsystem(elevatorSubsystem);

    // Sets the Joystick/Physical Driver Station ports, change port order in Driver Station to the numbers below.
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort); // 0
    private final Joystick driverstationTop = new Joystick(OIConstants.kTopHalfButtonBoardPort); // 1
    private final Joystick driverstationBottom = new Joystick(OIConstants.kBottomHalfButtonBoardPort); // 2
    // private final Joystick debug_secondary = new Joystick(4);

    // Sends a dropdown for us to choose an auto in the Dashboard.
    private final SendableChooser<Command> autoChooser;

    private final PhotonCamera leftvisionCamera = new PhotonCamera("ArducamLeft");
    // private final PhotonCamera rightvisionCamera = new PhotonCamera("ArducamRight");

    /*
    Assigns raw inputs on whichever joystick you're using into buttons we use to control the robot.
    Feel free to change the names if you decide to change the controller to a non-PS4 controller for clarity sake.
    Check Driver Station for buttonNumbers, they'll be in the USB order tab.
    */
    Trigger X1 = new JoystickButton(driverJoystick, 1);
    Trigger O2 = new JoystickButton(driverJoystick, 2);
    Trigger Square3 = new JoystickButton(driverJoystick, 3);
    Trigger Triangle4 = new JoystickButton(driverJoystick, 4);
    Trigger leftShoulder5 = new JoystickButton(driverJoystick, 5);
    Trigger rightShoulder6 = new JoystickButton(driverJoystick, 6);
    Trigger leftTrigger7 = new JoystickButton(driverJoystick, 7);
    Trigger rightTrigger8 = new JoystickButton(driverJoystick, 8);
    Trigger leftStickPress9 = new JoystickButton(driverJoystick, 9);
    Trigger rightStickPress10 = new JoystickButton(driverJoystick, 10);
    
    Trigger dPadNorth = new POVButton(driverJoystick, 0);
    Trigger dPadSouth = new POVButton(driverJoystick, 180);
    Trigger dPadWest = new POVButton(driverJoystick, 270);
    Trigger dPadEast = new POVButton(driverJoystick, 90);

    Trigger buttonT1 = new JoystickButton(driverstationTop, 1);
    Trigger buttonT2 = new JoystickButton(driverstationTop, 2);
    Trigger buttonT3 = new JoystickButton(driverstationTop, 3);
    Trigger buttonT4 = new JoystickButton(driverstationTop, 4);
    Trigger buttonT5 = new JoystickButton(driverstationTop, 5);
    Trigger buttonT6 = new JoystickButton(driverstationTop, 6);
    Trigger buttonT7 = new JoystickButton(driverstationTop, 7);
    Trigger buttonT8 = new JoystickButton(driverstationTop, 8);
    Trigger buttonT9 = new JoystickButton(driverstationTop, 9);
    Trigger buttonT10 = new JoystickButton(driverstationTop, 10);

    Trigger buttonB1 = new JoystickButton(driverstationBottom, 1);
    Trigger buttonB2 = new JoystickButton(driverstationBottom, 2);
    Trigger buttonB3 = new JoystickButton(driverstationBottom, 3);
    Trigger buttonB4 = new JoystickButton(driverstationBottom, 4);
    Trigger buttonB5 = new JoystickButton(driverstationBottom, 5);
    Trigger buttonB6 = new JoystickButton(driverstationBottom, 6);
    Trigger buttonB7 = new JoystickButton(driverstationBottom, 7);
    Trigger buttonB8 = new JoystickButton(driverstationBottom, 8);
    Trigger buttonB9 = new JoystickButton(driverstationBottom, 9);
    Trigger buttonB10 = new JoystickButton(driverstationBottom, 10);

    // Trigger buttonD7 = new JoystickButton(debug_secondary, 7);
    // Trigger buttonD8 = new JoystickButton(debug_secondary, 8);
    // Trigger buttonD9 = new JoystickButton(debug_secondary, 9);

    public RobotContainer() {
        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                        () -> driverJoystick.getY() * -1,
                                                                        () -> driverJoystick.getX() * -1)
                                                                    .withControllerRotationAxis(() -> driverJoystick.getRawAxis(2))
                                                                    .deadband(0.05D)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverJoystick::getX,
                                                                                                    driverJoystick::getY)
                                                                .headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                    .allianceRelativeControl(false);
        Command driveFieldOrientedDirectAngle      = swerveSubsystem.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity  = swerveSubsystem.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(
            driveDirectAngle);
        
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

        configureButtonBindings();
        
        // Use this line to add commands to PathPlanner, make sure to get spelling correct.
        // NamedCommands.registerCommand("ResetHeading", new ResetHeading(swerveSubsystem));
        // NamedCommands.registerCommand("ElevatorL0", new ElevatorZero(elevatorSubsystem,0));
        // NamedCommands.registerCommand("ElevatorL1", new AutoElevatorAbsolutePosition(elevatorSubsystem,1));
        // NamedCommands.registerCommand("ElevatorL2", new AutoElevatorAbsolutePosition(elevatorSubsystem,6));
        // NamedCommands.registerCommand("ElevatorL3", new AutoElevatorAbsolutePosition(elevatorSubsystem,13));
        // NamedCommands.registerCommand("ElevatorL4", new AutoElevatorAbsolutePosition(elevatorSubsystem,25));
        NamedCommands.registerCommand("Intake", new IntelligentIntake(intakeSubsystem,-.50));
        NamedCommands.registerCommand("Shoot", new SpinIntake(intakeSubsystem,-.50));
        NamedCommands.registerCommand("L4 Shoot", new SpinIntake(intakeSubsystem,-.40));
        NamedCommands.registerCommand("Eject", new SpinIntake(intakeSubsystem,.50));
        // NamedCommands.registerCommand("Auto Shoot", new AutoShoot(intakeSubsystem, ledSubsystem, -.50));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Sends a dropdown for us to choose an auto in the Dashboard.
        SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  

    private void configureButtonBindings() {
        /* 
        Used to set all Button Bindings as the name suggests, excluding moving the robot with the joystick,
        which is set with the Command Scheduler.
        */

        //  X1.onTrue(new ResetHeading(swerveSubsystem));
        //  O2.onTrue(new ResyncEncoders(swerveSubsystem)); 
         //Square3.whileTrue(new PathPlannerReefLineup(swerveSubsystem,false));
        //Triangle4.whileTrue(new BasicReefLineup(swerveSubsystem, () -> leftvisionCamera.getLatestResult(), "left", true));
         //leftShoulder5.onTrue(new ElevatorShift(elevatorSubsystem, -0.5));
         //rightShoulder6.onTrue(new ElevatorShift(elevatorSubsystem, 0.5));
        // leftTrigger7.whileTrue(new SpinIntake(intakeSubsystem, 0.5));
        // rightTrigger8.whileTrue(new SpinIntake(intakeSubsystem, -0.5));
        // leftStickPress9.onTrue(new);
        // rightStickPress10.onTrue(new);
        //dPadNorth.onTrue(new);
        // dPadEast.onrue(new);
        // dPadSouth.onTrue(new);
        // dPadWest.onTrue(new);

        //  buttonT1.onTrue(new ElevatorZero(elevatorSubsystem, 0)); // Brings elevator to 0
        //  buttonT2.onTrue(new ElevatorAbsolutePosition(elevatorSubsystem, 1)); // L1
        //  buttonT3.onTrue(new ElevatorAbsolutePosition(elevatorSubsystem, 6)); // L2
        //  buttonT4.onTrue(new ElevatorAbsolutePosition(elevatorSubsystem, 13.5)); // L3
        //  buttonT5.onTrue(new ElevatorAbsolutePosition(elevatorSubsystem, 25)); // L4
        //  buttonT6.whileTrue(new IntelligentIntake(intakeSubsystem, -.50)); // Smart Intake
         //buttonT7.whileTrue(new SpinIntake(intakeSubsystem, -.50));
        //  buttonT8.whileTrue(new SpinIntake(intakeSubsystem, -.50));
        //  buttonT9.whileTrue(new SpinIntake(intakeSubsystem, -.40));
        //  buttonT10.whileTrue(new SpinIntake(intakeSubsystem, .50));

        //  buttonB1.whileTrue(new SpinIntake(intakeSubsystem, -.40));
        // buttonB2.onTrue(new);
        // buttonB3.onTrue(new);
        // buttonB4.onTrue(new);
        // buttonB5.onTrue(new);
        //  buttonB6.onTrue(new ResetElevator(elevatorSubsystem));
        //  buttonB7.onTrue(new ApplyOffsets(swerveSubsystem));
        // buttonB8.onTrue(new);
        // buttonB8.onTrue(new);
        //  buttonB9.onTrue(new ResetHeading(swerveSubsystem));
        //  buttonB10.onTrue(new ResyncEncoders(swerveSubsystem));

        // // buttonD7.onTrue(new);
        // // buttonD8.onTrue(new);
        // // buttonD9.onTrue(new);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
  }
    
}
//??. .??