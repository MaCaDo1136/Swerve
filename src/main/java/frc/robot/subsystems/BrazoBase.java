package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.BrazoConstants;

public class BrazoBase {

 
    private final String motorName;

    private final CANSparkMax brazoMotor;

    private final RelativeEncoder brazoRelativeEncoder;
    private final AnalogInput absoluteEncoderBrazo;

    private final CANCoder encoderBrazo;

    private final PIDController brazoPIDController;
    
    public BrazoBase (String motorName, int brazoMotorId, int brazoEncoderId, int absoluteEncoderBrazoId) {
        this.motorName = motorName;

        brazoMotor = new CANSparkMax(brazoMotorId, MotorType.kBrushless);

        encoderBrazo = new CANCoder(brazoEncoderId);

        brazoRelativeEncoder = brazoMotor.getEncoder();
        
        absoluteEncoderBrazo = (AnalogInput) new edu.wpi.first.wpilibj.AnalogInput(absoluteEncoderBrazoId);

        brazoPIDController = new PIDController(BrazoConstants.kPBrazo, 0, 0);
        brazoPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

   

    private void resetEncoders() {
        brazoMotor.getEncoder().setPosition(0);
        System.out.println("Encoder reseted");
    }

    public double getBrazoAngle() {
        return brazoRelativeEncoder.getPosition();
    }
    
}
