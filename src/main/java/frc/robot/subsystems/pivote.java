package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.PivoteConstants;


public class pivote {
    
    private final String motorString = "pivote"; //nombre del motor
    private final CANSparkMax pivoteMotor; //motor
    private final AbsoluteEncoder pivoteAbsoluteEncoder = new AbsoluteEncoder(PivoteConstants.kPivoteAbsoluteEncoderId); //encoder
        
    private final PIDController pivotePID = new PIDController(PivoteConstants.kPivoteP, PivoteConstants.kPivoteI, PivoteConstants.kPivoteD); //PID

        //constructor
    public pivote (String motorString, int pivoteMotorId, int pivoteAbsoluteEncoderId, double pivoteP, double pivoteI, double pivoteD) {
        this.motorString = motorString;

        pivoteMotor = new CANSparkMax(PivoteConstants.kPivoteMotorId, MotorType.kBrushless); //motor
        pivoteAbsoluteEncoder = new AbsoluteEncoder() {
            
        };
    }

    public static final pivote Pivote(
        "pivote",
        PivoteConstants.kPivoteMotorId
        PivoteConstants.kPivoteAbsoluteEncoderId,
        PivoteConstants.kPivoteP,
        PivoteConstants.kPivoteI,
        PivoteConstants.kPivoteD);

    public double getPivotePosition() {
        return pivoteAbsoluteEncoder.getPosition();
    }
    public double getPivoteVelocity() {
        return pivoteAbsoluteEncoder.getVelocity();
    }
    public void stopPivote() {
        pivoteMotor.set(0);
    }

}