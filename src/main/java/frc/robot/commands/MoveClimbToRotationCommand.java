package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;

public class MoveClimbToRotationCommand extends Command {

    private final Climb m_climb;
    private final double position;
         
    public MoveClimbToRotationCommand(Climb climb, double _position) {
        m_climb = climb;
        position = _position;

        addRequirements(climb);
    }
        

    @Override
    public void initialize() {
        m_climb.setRotation(position);
    }

    @Override
    public boolean isFinished() {
        // End the command when the elevator reaches the desired position
        return Math.abs(m_climb.getRotation() - position) < 0.003;
    }
}

