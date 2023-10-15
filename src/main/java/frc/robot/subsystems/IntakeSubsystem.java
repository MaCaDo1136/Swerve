package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    
    boolean ButtonA = driver2Controller.getAButton();
    boolean ButtonB = driver2Controller.getBButton();

    if (ButtonA == true ){
      intakeMotor.set(1);
    }else if (ButtonB == true){
      intakeMotor.set(-1);
    }else if (ButtonA == false && ButtonB == false){
      intakeMotor.set(0);
    }
    }
  
  final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
  final XboxController driver2Controller = new XboxController(ControllerConstants.kDriver2ControllerPort);

  @Override
  public void periodic() {

  }

public void stopIntake() {
  intakeMotor.set(0);
  }
}
