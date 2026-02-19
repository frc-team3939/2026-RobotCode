package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private SparkMax driveMotor;
    private SparkMax steerMotor;
    private CANcoder    absoluteEncoder;
    private SparkClosedLoopController drivingPIDController;
    private SparkClosedLoopController turningPIDController;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder steerEncoder;
    
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID)
    {
        driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorCANID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(cancoderCANID);
        
        // Get the PID Controllers
        drivingPIDController = driveMotor.getClosedLoopController();
        turningPIDController = steerMotor.getClosedLoopController();
        
        // Get the encoders
        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();
        
        // Reset everything to factory default
        //driveMotor.();
        //steerMotor.restoreFactoryDefaults();
        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
        
        // Continue configuration here..
        
        // CANcoder Configuration
        CANcoderConfigurator cfg = absoluteEncoder.getConfigurator();
        cfg.apply(new CANcoderConfiguration());
        MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
        cfg.refresh(magnetSensorConfiguration);
        cfg.apply(magnetSensorConfiguration
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        // Steering Motor Configuration
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        driveConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        driveConfig.encoder
            .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
            .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        driveConfig.closedLoop
             .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
             .pidf(1.0, 0.0, 0.0, 0.0);

        driveMotor.configure(driveConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

        SparkMaxConfig steerConfig = new SparkMaxConfig();

        steerConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        steerConfig.encoder
            .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
            .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        steerConfig.closedLoop
             .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
             .pidf(1.0, 0.0, 0.0,0.0)
             .positionWrappingEnabled(true)
             .positionWrappingMaxInput(90)
             .positionWrappingMinInput(0);

        steerMotor.configure(steerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

          
        driveEncoder.setPosition(0);
        steerEncoder.setPosition(absoluteEncoder.getAbsolutePosition().refresh().getValue().in(Radians));
    }
    
    
    /**
    Get the distance in meters.
    */
    public double getDistance()
    {
        return driveEncoder.getPosition();
    }
    
    /**
    Get the angle.
    */
    public Rotation2d getAngle()
    {
          return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }
    
    /**
    Set the swerve module state.
    @param state The swerve module state to set.
    */
    public void setState(SwerveModuleState state)
    {
          turningPIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
          drivingPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    }

}
