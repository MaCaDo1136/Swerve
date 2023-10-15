package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivoteConstants;
import frc.robot.commands.PivoteCmd;

public class PivoteSubsystem extends SubsystemBase {
  // Creates a new PivoteSubsystem.
  public PivoteSubsystem() {
    
  final CANSparkMax pivoteMotor = new CANSparkMax(PivoteConstants.kPivoteMotorId, MotorType.kBrushless);
  final AbsoluteEncoder pivoteAbsoluteEncoder = pivoteMotor.getAbsoluteEncoder(Type.kDutyCycle);
  final PIDController pivotePIDController = new PIDController(PivoteConstants.kPivoteP, PivoteConstants.kPivoteI, PivoteConstants.kPivoteD);
  
  final XboxController driver2Controller = new XboxController(1);

  }
  public void stopPivote() {
    final CANSparkMax pivoteMotor = new CANSparkMax(PivoteConstants.kPivoteMotorId, MotorType.kBrushless);
    pivoteMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
