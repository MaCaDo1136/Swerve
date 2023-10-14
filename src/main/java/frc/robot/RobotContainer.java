package frc.robot;

import frc.robot.commands.swerveJoysticksCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final XboxController control1 = new XboxController(0);
  double lxJoystick = control1.getLeftX();
  double lyJoystick = control1.getLeftY();
  double rxJoystick = control1.getRightX();
  double ryJoystick = control1.getRightY();


  public RobotContainer() {
    // joysticks control 1
    swerveSubsystem.setDefaultCommand(new swerveJoysticksCmd(
        swerveSubsystem,
        () -> control1.getLeftX(),
        () -> control1.getLeftY(),
        () -> control1.getRightX(),
        () -> control1.getLeftStickButtonPressed()));
    configureBindings();   
  }


  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.waitSeconds(1);
    // An example command will be run in autonomous
   /*  return Autos.exampleAuto(m_exampleSubsystem); */
  }
}
