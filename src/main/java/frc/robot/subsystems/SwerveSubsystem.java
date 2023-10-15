package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    public static final SwerveModule frontLeft = new SwerveModule(
            4,
            "Front Left", 
            DriveConstants.kFrontLeftDriveMotorId,
            DriveConstants.kFrontLeftSteeringMotorId, 
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftSteeringEncoderReversed,
            DriveConstants.kFrontLeftDriveCANCoderId,
            DriveConstants.kFrontLeftDriveCANCoderOffsetDegrees,
            DriveConstants.kFrontLeftDriveCANCoderReversed
            );

    public static final SwerveModule frontRight = new SwerveModule(
            1,
            "Front Right", 
            DriveConstants.kFrontRightDriveMotorId,
            DriveConstants.kFrontRightSteeringMotorId, 
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightSteeringEncoderReversed,
            DriveConstants.kFrontRightDriveCANCoderId,
            DriveConstants.kFrontRightDriveCANCoderOffsetDegrees,
            DriveConstants.kFrontRightDriveCANCoderReversed
            );

    private static final SwerveModule backLeft = new SwerveModule(
            3,
            "Back Left", 
            DriveConstants.kBackLeftDriveMotorId,
            DriveConstants.kBackLeftSteeringMotorId, 
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftSteeringEncoderReversed,
            DriveConstants.kBackLeftDriveCANCoderId,
            DriveConstants.kBackLeftDriveCANCoderOffsetDegrees,
            DriveConstants.kBackLeftDriveCANCoderReversed
            );

    private static final SwerveModule backRight = new SwerveModule(
            2,
            "Back Right", 
            DriveConstants.kBackRightDriveMotorId,
            DriveConstants.kBackRightSteeringMotorId,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightSteeringEncoderReversed,
            DriveConstants.kBackRightDriveCANCoderId,
            DriveConstants.kBackRightDriveCANCoderOffsetDegrees,
            DriveConstants.kBackRightDriveCANCoderReversed
            );

 

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    // reseteo de la navx
    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getSwerveAngle() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public static void setModulesStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
