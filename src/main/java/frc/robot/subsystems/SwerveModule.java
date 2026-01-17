package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;

    private final CANcoder CANCoder;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turningConfig;

    private final String absoluteEncoderKey;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, String absoluteEncoderKey, boolean absoluteEncoderReversed) {

        this.absoluteEncoderKey = absoluteEncoderKey;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);
        CANCoder = new CANcoder(absoluteEncoderId, "rio");
        var toApply = new CANcoderConfiguration();

        Preferences.initDouble("FL-Offset", Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad);
        Preferences.initDouble("FR-Offset", Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad);
        Preferences.initDouble("BL-Offset", Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad);
        Preferences.initDouble("BR-Offset", Constants.DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad);

        /* User can change the configs if they want, or leave it empty for factory-default */
        CANCoder.getConfigurator().apply(toApply);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        
        driveConfig = new SparkMaxConfig();
        turningConfig = new SparkMaxConfig();

        driveConfig.inverted(driveMotorReversed);
        turningConfig.inverted(turningMotorReversed);
        turningConfig.smartCurrentLimit(20);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = CANCoder.getPosition().getValueAsDouble();
        angle = angle * 2 * Math.PI;
        angle *= (absoluteEncoderReversed ? -1.0 : 1.0);
        return angle + Preferences.getDouble(absoluteEncoderKey, 0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state.optimize(getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}