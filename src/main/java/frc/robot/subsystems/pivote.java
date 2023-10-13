package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.PivoteConstants;

public class pivote {
    
    private final String motorString = "pivote"; //nombre del motor
    private final CANSparkMax pivoteMotor = new CANSparkMax(PivoteConstants.kPivoteMotorId, MotorType.kBrushless); //motor
    private final AnalogInput pivoteAbsoluteEncoder = (AnalogInput) new edu.wpi.first.wpilibj.AnalogInput(PivoteConstants.kPivoteAbsoluteEncoderId); //encoder
    private final PIDController pivotePID = new PIDController(PivoteConstants.kPivoteP, PivoteConstants.kPivoteI, PivoteConstants.kPivoteD); //PID
    
    

}