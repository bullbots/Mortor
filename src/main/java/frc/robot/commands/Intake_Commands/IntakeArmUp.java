package frc.robot.commands.Intake_Commands;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeArmUp extends Command {
    private Intake intake;

    public IntakeArmUp(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // intake.getRaiseLowerSpinner().getPIDController().setReference(Constants.INTAKE_ARM_UP, CANSparkBase.ControlType.kSmartMotion);
        intake.getRaiseLowerSpinner().set(Constants.INTAKE_ARM_UP);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
