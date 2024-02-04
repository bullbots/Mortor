package frc.robot.commands.Intake_Commands;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Creates a new IntakeCargo
 */
public class IntakeCargos extends Command {

    private Intake m_intake;
    private double m_intakeVel;
    private double m_armVel;
    private Shooter m_shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    public IntakeCargos(Intake intake, double intakeVel, double armVel, Shooter shooter) { 
        m_intake = intake;
        m_intakeVel = intakeVel;
        m_armVel = armVel;
        m_shooter = shooter;
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override 
    public void initialize() {
        m_intake.getRaiseLowerSpinner().getPIDController().setReference(Constants.INTAKE_ARM_DOWN, CANSparkBase.ControlType.kSmartMotion);

        m_intake.getIntakeSpinner().set(m_intakeVel);
        m_intake.getArmSpinner().set(m_armVel);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override 
    public void execute() {
        if (m_shooter.getShooterSpinner().getStatorCurrent() > 50) {
            m_intake.getIntakeSpinner().stopMotor();
        } else {
            m_intake.getIntakeSpinner().set(m_intakeVel);
        }

    }

    @Override 
    public void end(boolean interrupted) {
        m_intake.stop();
        m_intake.getRaiseLowerSpinner().getPIDController().setReference(Constants.INTAKE_ARM_HOLD, CANSparkBase.ControlType.kSmartMotion);

    }

    @Override 
    public boolean isFinished() { return false; }
}
