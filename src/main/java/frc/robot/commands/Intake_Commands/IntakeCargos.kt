package frc.robot.commands.Intake_Commands

import com.revrobotics.CANSparkMax
import com.revrobotics.ControlType
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants

import frc.robot.subsystems.Intake
/**
 * Creates a new IntakeCargo
 */
class IntakeCargos(private var intake: Intake, var intakeVel: Double) : CommandBase() {
    // Use addRequirements() here to declare subsystem dependencies.
    init { addRequirements(intake) }

    // Called when the command is initially scheduled.
    override fun initialize() {
        intake.raiseLowerSpinner.pidController.setReference(Constants.INTAKE_ARM_DOWN, CANSparkMax.ControlType.kSmartMotion)
        intake.set(intakeVel)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {}

    override fun end(interrupted: Boolean) {
        intake.stop()
        intake.raiseLowerSpinner.pidController.setReference(Constants.INTAKE_ARM_HOLD, CANSparkMax.ControlType.kSmartMotion)

    }

    override fun isFinished(): Boolean { return false }
}