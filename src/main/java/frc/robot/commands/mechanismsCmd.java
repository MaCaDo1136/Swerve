package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class mechanismsCmd extends CommandBase {
    double lTrigger;
    double rTrigger;
    int aButton;
    int bButton;

    public mechanismsCmd (double lTrigger, double rTrigger, boolean aButton, boolean bButton){
        XboxController driver2Controller = new XboxController(1);

        lTrigger = driver2Controller.getLeftTriggerAxis();
        rTrigger = driver2Controller.getRightTriggerAxis();

        aButton = driver2Controller.getAButton();
        bButton = driver2Controller.getBButton();
    }
    
}
