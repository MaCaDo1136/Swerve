package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
        public static class ModuleConstants {
                public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
                public static final double kDriveMotorGearRatio = 14 / 15;
                public static final double kSteeringMotorGearRatio = 10 / 9;
                public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI
                                * kWheelDiameterMeters;
                public static final double kSteeringEncoderRot2Rad = kSteeringMotorGearRatio * 2 * Math.PI;
                public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
                public static final double kSteeringEncoderRPM2RadPerSec = kSteeringEncoderRot2Rad / 60;
                public static final double kPSteering = 0.5;
        }
        // hay que revisar esto

        public static final class DriveConstants {

                // distancia de las llantas de izq a derecha
                public static final double kTrackWidth = Units.inchesToMeters(20.75);
                // distancia de adelante para atras de las llantas
                public static final double kWheelBase = Units.inchesToMeters(20.75);

                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

                public static final int CANCoderID1 = 0;
                public static final int CANCoderID2 = 1;
                public static final int CANCoderID3 = 2;
                public static final int CANCoderID4 = 3;

                public static final int kFrontLeftDriveMotorPort = 4;
                public static final int kBackLeftDriveMotorPort = 5;
                public static final int kFrontRightDriveMotorPort = 6;
                public static final int kBackRightDriveMotorPort = 7;

                public static final int kFrontLeftSteeringMotorPort = 8;
                public static final int kBackLeftSteeringMotorPort = 9;
                public static final int kFrontRightSteeringMotorPort = 10;
                public static final int kBackRightSteeringMotorPort = 11;

                public static final boolean kFrontLeftSteeringEncoderReversed = true;
                public static final boolean kBackLeftSteeringEncoderReversed = true;
                public static final boolean kFrontRightSteeringEncoderReversed = true;
                public static final boolean kBackRightSteeringEncoderReversed = true;

                public static final boolean kFrontLeftDriveEncoderReversed = true;
                public static final boolean kBackLeftDriveEncoderReversed = true;
                public static final boolean kFrontRightDriveEncoderReversed = false;
                public static final boolean kBackRightDriveEncoderReversed = false;

                public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
                public static final int kBackLeftDriveAbsoluteEncoderPort = 13;
                public static final int kFrontRightDriveAbsoluteEncoderPort = 14;
                public static final int kBackRightDriveAbsoluteEncoderPort = 15;

                public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
                public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
                public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
                public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

                public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
                public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
                public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
                public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;

                public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
                public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

                public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
                public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
                public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
                public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        }

        public static final class AutoConstants {
                public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond
                                / 4;
                public static final double kMaxAngularSpeedRadiansPerSecond = //
                                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
                public static final double kPXController = 1.5;
                public static final double kPYController = 1.5;
                public static final double kPThetaController = 3;

                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                                new TrapezoidProfile.Constraints(
                                                kMaxAngularSpeedRadiansPerSecond,
                                                kMaxAngularAccelerationRadiansPerSecondSquared);
        }
        //constantes del brazo
        public static final class Brazo {

                public static final int CANCoderID5 = 16;
                public static final int kBrazoMotorPort = 17;

                public static final int kBrazoEncoderPort = 18;


        }
        // hay que checar esto por lo del control
        public static final class OIConstants {
                public static final int kDriverControllerPort = 0;

                public static final int kDriverYAxis = 1;
                public static final int kDriverXAxis = 0;
                public static final int kDriverRotAxis = 4;
                public static final int kDriverFieldOrientedButtonIdx = 1;

                public static final double kDeadband = 0.05;
        }
}
