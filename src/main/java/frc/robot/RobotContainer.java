package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;
import frc.robot.commands.*;
import frc.robot.commands.Intake.IntelligentIntake;
import frc.robot.commands.Intake.OscillateIntakeRPM;
import frc.robot.commands.Intake.SpinIntake;
import frc.robot.commands.Intake.SpinIntakeRPM;
import frc.robot.commands.Shooter.ShooterRPM;
import frc.robot.commands.Shooter.ShooterRPMDistance;

import static edu.wpi.first.units.Units.Radian;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private Command driveAim;


    // private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    // private final LEDSubsystem ledSubsystem = new LEDSubsystem(elevatorSubsystem);

    // Sets the Joystick/Physical Driver Station ports, change port order in Driver Station to the numbers below.
    private final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort); // 0
    private final Joystick driverstationTop = new Joystick(OIConstants.kTopHalfButtonBoardPort); // 1
    private final Joystick driverstationBottom = new Joystick(OIConstants.kBottomHalfButtonBoardPort); // 2
    // private final Joystick debug_secondary = new Joystick(4);

    // Sends a dropdown for us to choose an auto in the Dashboard.
    private final SendableChooser<Command> autoChooser;

    private final PhotonCamera leftvisionCamera = new PhotonCamera("ArducamLeft");
    private final PhotonCamera rightvisionCamera = new PhotonCamera("ArducamRight");

    /*
    Assigns raw inputs on whichever joystick you're using into buttons we use to control the robot.
    Feel free to change the names if you decide to change the controller to a non-PS4 controller for clarity sake.
    Check Driver Station for buttonNumbers, they'll be in the USB order tab.
    */
    // Trigger X1 = new JoystickButton(driverJoystick, 1);
    // Trigger O2 = new JoystickButton(driverJoystick, 2);
    // Trigger Square3 = new JoystickButton(driverJoystick, 3);
    // Trigger Triangle4 = new JoystickButton(driverJoystick, 4);
    // Trigger leftShoulder5 = new JoystickButton(driverJoystick, 5);
    // Trigger rightShoulder6 = new JoystickButton(driverJoystick, 6);
    // Trigger leftTrigger7 = new JoystickButton(driverJoystick, 7);
    // Trigger rightTrigger8 = new JoystickButton(driverJoystick, 8);
    // Trigger leftStickPress9 = new JoystickButton(driverJoystick, 9);
    // Trigger rightStickPress10 = new JoystickButton(driverJoystick, 10);
    
    // Trigger dPadNorth = new POVButton(driverJoystick, 0);
    //Trigger dPadSouth = new POVButton(driverJoystick, 180);
    //Trigger dPadWest = new POVButton(driverJoystick, 270);
    //Trigger dPadEast = new POVButton(driverJoystick, 90);

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

        NamedCommands.registerCommand("SpinIntake", new SpinIntake(intakeSubsystem, -5000));
        NamedCommands.registerCommand("Shoot", new ShooterRPMDistance(shooterSubsystem, feederSubsystem, swerveSubsystem, 1.0));
        NamedCommands.registerCommand("OscillateIntake", new OscillateIntakeRPM(intakeSubsystem, 2500));
        NamedCommands.registerCommand("Driveaim", driveAim);
        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                        () -> driverJoystick.getLeftY() * -1,
                                                                        () -> driverJoystick.getLeftX() * -1)
                                                                    .withControllerRotationAxis(() -> driverJoystick.getRightX())
                                                                    .deadband(0.05D)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverJoystick.getLeftX(),
                                                                                                   () ->  driverJoystick.getLeftY())
                                                                .headingWhile(true);

        Pose2d redHubLocation = new Pose2d(11.920,4.021,Rotation2d.kZero);
        Pose2d blueHubLocation = new Pose2d(4.612,4.021,Rotation2d.kZero);
    

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
         */
        
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                    .allianceRelativeControl(false);
        Supplier<Pose2d> hubSupplier = () -> {
            Pose2d hubLocation =(DriverStation.getAlliance().orElse(Alliance.Red)==Alliance.Blue)?blueHubLocation:redHubLocation;
            Pose2d robotAim = new Pose2d(hubLocation.getX(), hubLocation.getY(), hubLocation.getRotation().plus(Rotation2d.fromDegrees(90)));
            return robotAim;
        };
        SwerveInputStream hubAim = driveAngularVelocity.copy().aim(hubSupplier).aimHeadingOffset(Rotation2d.fromDegrees(180)).aimHeadingOffset(true).aimWhile(true);
        Command driveFieldOrientedDirectAngle      = swerveSubsystem.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity  = swerveSubsystem.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(
            driveDirectAngle);
        driveAim = swerveSubsystem.driveFieldOriented(hubAim);
    
        
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

        configureButtonBindings();
        
        // Use this line to add commands to PathPlanner, make sure to get spelling correct.
        // NamedCommands.registerCommand("ResetHeading", new ResetHeading(swerveSubsystem));
        // NamedCommands.registerCommand("ElevatorL0", new ElevatorZero(elevatorSubsystem,0));
        // NamedCommands.registerCommand("ElevatorL1", new AutoElevatorAbsolutePosition(elevatorSubsystem,1));
        // NamedCommands.registerCommand("ElevatorL2", new AutoElevatorAbsolutePosition(elevatorSubsystem,6));
        // NamedCommands.registerCommand("ElevatorL3", new AutoElevatorAbsolutePosition(elevatorSubsystem,13));
        // NamedCommands.registerCommand("ElevatorL4", new AutoElevatorAbsolutePosition(elevatorSubsystem,25));
        // NamedCommands.registerCommand("L4 Shoot", new SpinIntake(intakeSubsystem,-.40));
        // NamedCommands.registerCommand("Eject", new SpinIntake(intakeSubsystem,.50));
        // NamedCommands.registerCommand("SpinIntake", new SpinIntakeRPM(intakeSubsystem, 2000));
        // NamedCommands.registerCommand("Shoot", new ShooterRPMDistance(shooterSubsystem, feederSubsystem, swerveSubsystem, 1.0));
        // NamedCommands.registerCommand("OscillateIntake", new OscillateIntakeRPM(intakeSubsystem, 2500));

        //NamedCommands.registerCommand("Intake", new IntelligentIntake(intakeSubsystem,-.50));
        ///NamedCommands.registerCommand("Shoot", new SpinIntake(intakeSubsystem,-.50));
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
        
        driverJoystick.x().onTrue(new ResetHeading(swerveSubsystem));
        driverJoystick.a().toggleOnTrue(new SpinIntakeRPM(intakeSubsystem, -5000));
        //driverJoystick.rightBumper().whileTrue(new ShooterRPMDistance(shooterSubsystem, feederSubsystem, swerveSubsystem, 1.0).alongWith(new OscillateIntakeRPM(intakeSubsystem, 2000)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        //driverJoystick.y().whileTrue(new DistanceTest(swerveSubsystem, shooterSubsystem));
        //driverJoystick.leftTrigger().toggleOnTrue(driveAim);
        driverJoystick.leftTrigger().whileTrue(new ShooterRPMDistance(shooterSubsystem, feederSubsystem, swerveSubsystem, 1.0).alongWith(new OscillateIntakeRPM(intakeSubsystem, 3000)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        driverJoystick.rightTrigger().whileTrue(new ShooterRPMDistance(shooterSubsystem, feederSubsystem, swerveSubsystem, 1.0).alongWith(new OscillateIntakeRPM(intakeSubsystem, 3000)).alongWith((driveAim)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        driverJoystick.leftBumper().whileTrue(new SpinIntakeRPM(intakeSubsystem, -1000));
        //driverJoystick.rightBumper().whileTrue(new ShooterRPM(shooterSubsystem, feederSubsystem, -500, -500));
        driverJoystick.b().whileTrue(new ShooterRPM(shooterSubsystem, feederSubsystem, -2700, 5000).alongWith(new OscillateIntakeRPM(intakeSubsystem, 4000)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        //driverJoystick..whileTrue(new SpinBelt(feederSubsystem));

        // rightStickPress10.onTrue(new);
        //dPadNorth.onTrue(new);
        // dPadEast.onrue(new);
        // dPadSouth.onTrue(new);s
        // dPadWest.onTrue(new);

        //  buttonT1.onTrue(new ElevatorZero(elevatorSubsystem, 0)); // Brings elevator to 0
        //  buttonT2.onTrue(new ElevatorAbsolutePosition(elevatorSubsystem, 1)); // L1
        buttonT3.whileTrue(new ShooterRPMDistance(shooterSubsystem, feederSubsystem, swerveSubsystem, 0));
        //  buttonT4.onTrue(new ElevatorAbsolutePosition(elevatorSubsystem, 13.5)); // L3
        //  buttonT5.onTrue(new ElevatorAbsolutePosition(elevatorSubsystem, 25)); // L4
        buttonT6.whileTrue(feederSubsystem, -5000); 
        //                                                                     REVERSE FEEDER
        buttonT7.whileTrue(new SpinIntake(intakeSubsystem, 5000).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        //                                                                     REVERSE INTAKE
        //buttonT8.whileTrue(new SpinIntakeRPM(intakeSubsystem, 3000));
        buttonT9.whileTrue(new SpinIntake(intakeSubsystem, -5000).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        //                                                                     INTAKE BUTTON
        buttonT10.whileTrue(swerveSubsystem.lockPose());

        buttonB1.whileTrue(new SpinIntakeRPM(intakeSubsystem, -1000));
        //                                                                     CLEANING BUTTON
        buttonB2.whileTrue(new ShooterRPM(shooterSubsystem, feederSubsystem, -500, 500 ).alongWith(new SpinIntakeRPM(intakeSubsystem,-500)));
        //                                                                     TEST PATH BUTTON
        // buttonB3.onTrue(new);
        // buttonB4.onTrue(new);
        // buttonB5.onTrue(new);
        //  buttonB6.onTrue(new ResetElevator(elevatorSubsystem));
        //  buttonB7.onTrue(new ApplyOffsets(swerveSubsystem));
        // buttonB8.onTrue(new);
        // buttonB8.onTrue(new);
        // buttonB9.onTrue(new ResetHeading(swerveSubsystem));
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