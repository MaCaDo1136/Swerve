package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivoteConstants;
import frc.robot.commands.mechanismsCmd;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final CANSparkMax pivoteMotor = new CANSparkMax(PivoteConstants.kPivoteMotorId, MotorType.kBrushless); //motor del pivote
  private final AbsoluteEncoder pivoteAbsoluteEncoder = new AbsoluteEncoder(PivoteConstants.kPivoteAbsoluteEncoderId); //encoder del pivote
  private final PIDController pivotePID = new PIDController(PivoteConstants.kPivoteP, PivoteConstants.kPivoteI, PivoteConstants.kPivoteD); //PID
  
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless); //motor del intake

  private final XboxController driver2Controller = new XboxController(1);
  @Override
  public void robotInit() {
    pivoteMotor.setInverted(PivoteConstants.kPivoteInverted);
    //pivoteMotor.set(0.5);

    pivoteAbsoluteEncoder.getPosition();

    m_robotContainer = new RobotContainer();
  }

 
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Pivote Position", pivoteAbsoluteEncoder.getPosition());
  }


  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}


  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();


    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }


  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    //pivote

    double pivotePower = (- driver2Controller.getLeftTriggerAxis()) + (driver2Controller.getRightTriggerAxis());
    double pivotePosition = pivoteAbsoluteEncoder.getPosition();

    if (pivotePosition <= 1) {

    if (pivotePower >= 0.1 || pivotePower <= -0.1) {
      pivoteMotor.set(pivotePower);
    } else {
      pivoteMotor.set(0);
    }
  } else {
    pivoteMotor.set(-0.1);
  }

  //intake

  boolean aButton = driver2Controller.getAButton();
  boolean bButton = driver2Controller.getBButton();
  int intakePower = 0;

   if (aButton = true) {
    intakePower = 1;
   } else if (bButton = true) {
    intakePower = -1;
   }

   if (intakePower == 1) {
    intakeMotor.set(0.5);
   } else if (intakePower == -1) {
    intakeMotor.set(-0.5);
  } else {
    intakeMotor.set(0);
  }

  
  
  //cosas random
  if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }


  @Override
  public void teleopPeriodic() {
  
  }

  @Override
  public void testInit() {

    CommandScheduler.getInstance().cancelAll();
  }


  @Override
  public void testPeriodic() {}


  @Override
  public void simulationInit() {}


  @Override
  public void simulationPeriodic() {}
}
