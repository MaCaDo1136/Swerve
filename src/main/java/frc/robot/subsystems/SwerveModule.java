package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
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

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad; 


    public SwerveModule(int moduleNumber, String moduleName, int driveMotorId, int steeringMotorId, boolean driveMotorReversed, boolean steeringMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, int enconderId) {
        
        this.moduleNumber = moduleNumber;
        this.moduleName = moduleName;
     
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = (AnalogInput) new edu.wpi.first.wpilibj.AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        steeringMotor = new CANSparkMax(steeringMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        steeringMotor.setInverted(steeringMotorReversed);

        encoder1 = new CANCoder(enconderId);

        driveEncoder = driveMotor.getEncoder();
        steeringEncoder = steeringMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        steeringEncoder.setPositionConversionFactor(ModuleConstants.kSteeringEncoderRot2Rad);
        steeringEncoder.setVelocityConversionFactor(ModuleConstants.kSteeringEncoderRPM2RadPerSec);

        steeringPIDController = new PIDController(ModuleConstants.kPSteering, 0 , 0);
        steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
            
    }
    //meter los datos de los encoders para los angulos y velocidad
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
    public double getAbsoluteEncoderDegrees() {
        return encoder1.getAbsolutePosition();
    }
    //Reseteo de encoders
    public void resetEncoders() {
        driveEncoder.setPosition(0);    
        steeringEncoder.setPosition(getAbsoluteEncoderDegrees());
    }
    //PID
    public SwerveModuleState getState() { 
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteeringPosition()));
    }
    //actuate the module
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }

        //si es mas de 90 grados voltear para el otro lado
        state = SwerveModuleState.optimize(state, getState().angle);
        
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steeringMotor.set(steeringPIDController.calculate(getSteeringPosition(), state.angle.getDegrees()));
        //en la siguiente linea le pique a: add cast to 'absoluteEncoder' si no jala, nomas es de quitar: ((edu.wpi.first.wpilibj.AnalogInput) absoluteEncoder) y poner solo: absoluteEncoder
        SmartDashboard.putString("Swerve[" + ((edu.wpi.first.wpilibj.AnalogInput) absoluteEncoder).getChannel() + "] state", state.toString());
    }
    public void stop() {
        driveMotor.set(0);
        steeringMotor.set(0);
    }
}


