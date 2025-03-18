package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final SparkMax motor;          
    private final RelativeEncoder encoder;  
    private final PIDController pid;        
    
    private final double COUNTS_PER_INCH = 42; 
    
    public Arm() { 
        motor = new SparkMax(11, MotorType.kBrushless);
        encoder = motor.getEncoder();
        
 
        pid = new PIDController(20, 0, 0);
    }

    // Returns elevator height
    public double getRotation() {
        return encoder.getPosition() / COUNTS_PER_INCH;
    }

    public void setRotation(double targetRotation) {
        double output = pid.calculate(getRotation(), targetRotation);
        motor.set(output);  
    }
    public void resetRotation() {
        encoder.setPosition(0);
    }
    
    public void setPosition(double targetRotation) {
        pid.setSetpoint(targetRotation);
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Good place to update SmartDashboard values if needed
         double pidOutput = pid.calculate(getRotation());
        motor.set(pidOutput);
        SmartDashboard.putNumber("Position", getRotation());
    
    }

}