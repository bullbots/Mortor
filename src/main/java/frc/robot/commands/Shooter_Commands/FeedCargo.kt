package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake

class FeedCargo(private val intake: Intake, private val intakeVel: Double) : CommandBase() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intake)
    }

    override fun initialize() {
        intake.intakeSpinner.set(intakeVel)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        intake.intakeSpinner.stopMotor()
    }
}
