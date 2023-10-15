package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  private final Supplier<Boolean> buttonA;
  private final Supplier<Boolean> buttonB;

  public IntakeCmd(IntakeSubsystem intakeSubsystem, Supplier<Boolean> buttonA, Supplier<Boolean> buttonB) {
    this.intakeSubsystem = intakeSubsystem;
    this.buttonA = buttonA;
    this.buttonB = buttonB;

    addRequirements(intakeSubsystem);
  }


  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    //tomar datos de los botones para el intake
    boolean intakeIn = buttonA.get();
    boolean intakeOut = buttonB.get();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
