package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class swerveJoysticksCmd extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, steeringSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, steeringLimiter;

  public swerveJoysticksCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction, Supplier<Double> steeringSpdFunction, Supplier<Boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.steeringSpdFunction = steeringSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.steeringLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // toma de datos de los joysticks
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double steeringSpeed = steeringSpdFunction.get();

    // deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0;
    steeringSpeed = Math.abs(steeringSpeed) > OIConstants.kDeadband ? steeringSpeed : 0;

    // para que se sienta smooth y no matar los motores
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond;
    steeringSpeed = steeringLimiter.calculate(steeringSpeed)
        * DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;

    // velocidad del chasis
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
      // relativo a la cancha
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, steeringSpeed, swerveSubsystem.getSwerveAngle());
    } else {
      // relativo al frente del robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, steeringSpeed);
    }

    // velocidades de cada modulo
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // pasarlo a las llantas
    SwerveSubsystem.setModulesStates(moduleStates);

    System.out.println("HelloWorld");
  }


  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
