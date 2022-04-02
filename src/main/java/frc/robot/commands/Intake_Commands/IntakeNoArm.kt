package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

class IntakeNoArm(private val intake: Intake, private val shooter: Shooter) : CommandBase() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intake)
    }

    override fun initialize() {
        intake.intakeSpinner.set(0.6)
        shooter.shooterSpinner.set(-0.3)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        intake.intakeSpinner.set(0.0)
        shooter.shooterSpinner.set(0.0)
    }
}
