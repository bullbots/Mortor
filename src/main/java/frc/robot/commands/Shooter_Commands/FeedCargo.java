package frc.robot.commands.Shooter_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class FeedCargo extends CommandBase {

    private Intake m_intake;
    private double m_intakeVel;


    // each subsystem used by the command must be passed into the addRequirements() method
    public FeedCargo(Intake intake, double intakeVel) { 
        m_intake = intake;
        m_intakeVel = intakeVel;
        addRequirements(intake); 
    }

    @Override 
    public void initialize() { m_intake.getIntakeSpinner().set(m_intakeVel); }

    @Override 
    public void execute() {}

    @Override 
    public boolean isFinished() { return false; }

    @Override 
    public void end(boolean interrupted) { m_intake.getIntakeSpinner().stopMotor(); }
}
