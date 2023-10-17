package frc.robot;

import frc.robot.Constants.ControllerConstants;
//import frc.robot.commands.IntakeCmd;
//import frc.robot.commands.PivoteCmd;
import frc.robot.commands.swerveJoysticksCmd;
//import frc.robot.subsystems.PivoteSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
//swerve
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final XboxController control1 = new XboxController(ControllerConstants.kDriver1ControllerPort);
  double lxJoystick = control1.getLeftX();
  double lyJoystick = control1.getLeftY();
  double rxJoystick = control1.getRightX();
  double ryJoystick = control1.getRightY();

 
//mecanismos
 // private final PivoteSubsystem pivoteSubsystem = new PivoteSubsystem();
  //private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

 // private final XboxController control2 = new XboxController(ControllerConstants.kDriver2ControllerPort);
 // double lTrigger = control2.getLeftTriggerAxis();
 // double rTrigger = control2.getRightTriggerAxis();
 // boolean aButton = control2.getAButton();
  //boolean bButton = control2.getBButton();

  public RobotContainer() {
    // joysticks control 1
    swerveSubsystem.setDefaultCommand(new swerveJoysticksCmd(
        swerveSubsystem,
        () -> control1.getLeftX(),
        () -> control1.getLeftY(),
        
        () -> control1.getRightX(),
        () -> control1.getLeftStickButtonPressed()));
    configureBindings(); 
    
    System.out.println("HelloWorld");
    // Pivote control 2
   /*PivoteSubsystem.setDefaultCommand(new PivoteCmd(
      pivoteSubsystem,
      () -> control2.getLeftTriggerAxis(),
      () -> control2.getRightTriggerAxis()));
      configureBindings();
    
    // Intake control 2
     intakeSubsystem.setDefaultCommand(new IntakeCmd(
      intakeSubsystem,
      () -> control2.getAButton(),
      () -> control2.getBButton()));
      configureBindings();
      */
  }

 

  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.print("Hi");
    // An example command will be run in autonomous
   /*  return Autos.exampleAuto(m_exampleSubsystem); */
  }
}
