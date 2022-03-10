package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Intake

class IntakeArm(private var intake: Intake, private var armVel: Double = 0.5) : CommandBase() {

    init { addRequirements(intake) }

    override fun initialize() {
        intake.raiseLowerSpinner.set(armVel)
    }

    override fun execute() { }

    override fun end(interrupted: Boolean) {
        intake.stop()
    }

    override fun isFinished(): Boolean {
        return false
    }

}