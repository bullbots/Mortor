package frc.robot.commands.Intake_Commands;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class AutoArm extends Command {
    private double targetTraj = 0.0;
    private final Intake intake;
    private boolean isDown = false;

    public AutoArm(Intake intake, boolean isDown) {
        this.intake = intake;
        this.isDown = isDown;
        if (isDown) {
            targetTraj = Constants.INTAKE_ARM_DOWN;
        } else {
            targetTraj = Constants.INTAKE_ARM_HOLD;
        }
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("INFO: AutoArm.initialize");
        intake.getRaiseLowerSpinner().getPIDController().setReference(targetTraj, CANSparkBase.ControlType.kSmartMotion);
    }

    @Override
    public void execute() {
//        intake.raiseLowerSpinner.pidController.setReference(targetTraj, CANSparkMax.ControlType.kSmartMotion)
    }

    @Override
    public boolean isFinished() {
        var trajPoseError = targetTraj - intake.getRaiseLowerSpinner().getEncoder().getPosition();
        if (isDown) {
            return Math.abs(trajPoseError) < 1;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("INFO: AutoArm.end");
        intake.stopArm();
    }
}