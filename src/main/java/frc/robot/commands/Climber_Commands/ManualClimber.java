package frc.robot.commands.Climber_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ManualClimber extends Command {

    private final Climber climber;
    private final double climberVel;

    // Use addRequirements() here to declare subsystem dependencies.
    public ManualClimber(Climber climber, double climberVel) {
        this.climber = climber;
        this.climberVel = climberVel;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        System.out.println("INFO: ManualClimber.initialize");
    }

    @Override
    public void execute() {
        climber.setManual(climberVel);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}