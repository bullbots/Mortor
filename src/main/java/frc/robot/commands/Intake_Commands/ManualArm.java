package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ManualArm extends Command {

    private double armVel;
    private Intake intake;

    public ManualArm(Intake intake, double armVel) {
        this.armVel = armVel;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.getRaiseLowerSpinner().set(armVel);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.getRaiseLowerSpinner().set(0.0);
    }
}
