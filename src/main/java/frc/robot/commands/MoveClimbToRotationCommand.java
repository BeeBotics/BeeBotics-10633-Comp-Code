package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class MoveClimbToRotationCommand extends Command {

    private final Climb climb;
    private final double position;
         
    public MoveClimbToRotationCommand(Climb m_climb, double _position) {
        climb = m_climb;
        position = _position;

        addRequirements(climb);
    }
        

    @Override
    public void initialize() {
        climb.setRotation(position);
    }

    @Override
    public boolean isFinished() {
        // End the command when the climb reaches the desired position
        return Math.abs(climb.getRotation() - position) < 0.003;
    }
}

