package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Indexer extends CommandBase {

    private Shooter m_shooter;
    private double m_shooterVel;

    // each subsystem used by the command must be passed into the addRequirements() method
    public Indexer(Shooter shooter, double shooterVel) { 
        m_shooter = shooter;
        m_shooterVel = shooterVel;
        addRequirements(shooter); 
    }

    @Override 
    public void initialize() { m_shooter.getShooterSpinner().set(m_shooterVel); }

    @Override 
    public void execute() {}


    @Override 
    public boolean isFinished() { return false; }

    @Override 
    public void end(boolean interrupted) { m_shooter.getShooterSpinner().stopMotor(); }
}
