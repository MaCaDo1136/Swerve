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

public class Brazo {
 
    private final CANSparkMax brazoMotor;

    private final RelativeEncoder brazoEncoder;

    private final CANCoder encoderBrazo;

    private final PIDController brazoPIDController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    
    public Brazo () {
        brazoMotor = new CANSparkMax(17, MotorType.kBrushless);

        brazoEncoder = brazoMotor.getEncoder();

        brazoEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        brazoEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        brazoPIDController = new PIDController(ModuleConstants.kPSteering, 0 , 0);
        brazoPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

   

    private void resetEncoders() {
        brazoMotor.getEncoder().setPosition(0);
    }

    public double getBrazoAngle() {
        return brazoEncoder.getPosition();
    }
    
}
