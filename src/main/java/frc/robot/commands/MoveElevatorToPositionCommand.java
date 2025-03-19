package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator;

public class MoveElevatorToPositionCommand extends Command {

    private final elevator elevator;
    private final double position;
         
    public MoveElevatorToPositionCommand(elevator m_elevator, double _position) {
        elevator = m_elevator;
        position = _position;

        addRequirements(elevator);
    }
        

    @Override
    public void initialize() {
        elevator.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        // End the command when the elevator reaches the desired position
        return Math.abs(elevator.getHeight() - position) < 0.1;
    }
}


