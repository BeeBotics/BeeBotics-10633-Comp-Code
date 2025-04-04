package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {
    private final SparkMax motor;          
    private final RelativeEncoder encoder;  
    private final PIDController pid;        
    
    private final double COUNTS_PER_INCH = 42; 
    
    public elevator() { 
        motor = new SparkMax(10, MotorType.kBrushless);
        encoder = motor.getEncoder();
 
        pid = new PIDController(4, 0, 0.4);
    }

    // Returns elevator height
    public double getHeight() {
        return encoder.getPosition() / COUNTS_PER_INCH;
    }
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public void setPosition(double targetHeight) {
        pid.setSetpoint(targetHeight);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Good place to update SmartDashboard values if needed
        double pidOutput = pid.calculate(getHeight());
        motor.set(pidOutput);
        SmartDashboard.putNumber("Position", getHeight());
        // SmartDashboard.putNumber("Voltage", pidOutput);
        
     }
}