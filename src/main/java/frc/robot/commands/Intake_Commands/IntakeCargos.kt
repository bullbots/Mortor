package frc.robot.commands.Intake_Commands

import com.revrobotics.CANSparkMax
import com.revrobotics.ControlType
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants

import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

/**
 * Creates a new IntakeCargo
 */
class IntakeCargos(private val intake: Intake, private val intakeVel: Double, private val armVel: Double, private val shooter: Shooter) : CommandBase() {

    // Use addRequirements() here to declare subsystem dependencies.
    init { addRequirements(intake) }

    // Called when the command is initially scheduled.
    override fun initialize() {
        intake.raiseLowerSpinner.pidController.setReference(Constants.INTAKE_ARM_DOWN, CANSparkMax.ControlType.kSmartMotion)

        intake.intakeSpinner.set(intakeVel)
        intake.armSpinner.set(armVel)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        if (shooter.shooterSpinner.statorCurrent > 50) {
            intake.intakeSpinner.stopMotor()
        } else {
            intake.intakeSpinner.set(intakeVel)
        }

    }

    override fun end(interrupted: Boolean) {
        intake.stop()
        intake.raiseLowerSpinner.pidController.setReference(Constants.INTAKE_ARM_HOLD, CANSparkMax.ControlType.kSmartMotion)

    }

    override fun isFinished(): Boolean { return false }
}