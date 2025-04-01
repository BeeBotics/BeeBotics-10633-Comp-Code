package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final SparkMax motor;          
    private final RelativeEncoder encoder;  
    private final PIDController pid;        
    
    private final double COUNTS_PER_INCH = 42; 
    
    public Climb () { 
        motor = new SparkMax(20, MotorType.kBrushless);
        encoder = motor.getEncoder();
 
        pid = new PIDController(3, 0, 0);
    }

    // Returns elevator height
    public double getRotation() {
        return encoder.getPosition() / COUNTS_PER_INCH;
    }
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public void setRotation(double targetHeight) {
        pid.setSetpoint(targetHeight);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Good place to update SmartDashboard values if needed
        double pidOutput = pid.calculate(getRotation());
        motor.set(pidOutput);
        SmartDashboard.putNumber("Climb Rotation", getRotation());

     }
}