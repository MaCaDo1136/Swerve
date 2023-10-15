package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final int moduleNumber;
    private final String moduleName;

    private final CANSparkMax driveMotor;
    private final CANSparkMax steeringMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steeringEncoder;
    private final CANCoder encoder1;

    private final PIDController steeringPIDController;


    public SwerveModule(int moduleNumber, String moduleName, int driveMotorId, int steeringMotorId,
            boolean driveMotorReversed, boolean steeringMotorReversed, int CANCoderId, double CANCoderOffset,
            boolean CANCoderReversed) {

        this.moduleNumber = moduleNumber;
        this.moduleName = moduleName;

        CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
        cancoderConfig.magnetOffsetDegrees = CANCoderOffset;
        cancoderConfig.sensorDirection = CANCoderReversed;
        cancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        encoder1 = new CANCoder(CANCoderId);
        encoder1.configAllSettings(cancoderConfig);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        steeringMotor = new CANSparkMax(steeringMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        steeringMotor.setInverted(steeringMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        steeringEncoder = steeringMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        steeringEncoder.setPositionConversionFactor(ModuleConstants.kSteeringEncoderRot2Degrees);
        steeringEncoder.setVelocityConversionFactor(ModuleConstants.kSteeringEncoderRPM2DegreesPerSec);

        steeringPIDController = new PIDController(ModuleConstants.kPSteering, 0, 0);
        steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

    }

    public double getModuleNumber() {
        return moduleNumber;
    }

    public String getModuleName() {
        return moduleName;
    }

    // meter los datos de los encoders para los angulos y velocidad
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getSteeringPosition() {
        return steeringEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getSteeringVelocity() {
        return steeringEncoder.getVelocity();
    }

    public double getCANCoderDegrees() {
        return encoder1.getPosition();
    }

    /*
     * Resets all encoder positions. Due to an internal error in the CANcoder API,
     * you must wait less than a second in order for the cancoder to boot to
     * absolute position mode, otherwise bad initial measurements will be given
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        try {
            Thread.sleep(700);
        } catch (Exception e) {

        }
        steeringEncoder.setPosition(getCANCoderDegrees());
    }

    // PID
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteeringPosition()));
    }

    // actuate the module
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // si es mas de 90 grados voltear para el otro lado
        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steeringMotor.set(steeringPIDController.calculate(getSteeringPosition(), state.angle.getDegrees()));

        SmartDashboard.putString("Swerve[" + (encoder1.getDeviceID()) + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        steeringMotor.set(0);
    }
}
