package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToRotationCommand extends Command {

    private final Arm arm;
    private final double position;
         
    public MoveArmToRotationCommand(Arm m_arm, double _position) {
        arm = m_arm;
        position = _position;

        addRequirements(arm);
    }
        

    @Override
    public void initialize() {
        arm.setRotation(position);
    }

    @Override
    public boolean isFinished() {
        // End the command when the arm reaches the desired position
        return Math.abs(arm.getRotation() - position) < 0.003;
    }
}

