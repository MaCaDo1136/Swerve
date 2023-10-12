package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.BrazoConstants;

public class BrazoSubsystem extends BrazoBase {
    public static final BrazoBase brazo = new BrazoBase(
        "Brazo",
        BrazoConstants.kBrazoMotorId,
        BrazoConstants.CANCoderID5,
        BrazoConstants.kBrazoAbsoluteEncoderId);

    
}
