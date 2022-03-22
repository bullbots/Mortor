package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Intake

class HoldArmCommand(private val intake: Intake) : CommandBase() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intake)

    }

    override fun initialize() {}

    override fun execute() {
        if (intake.raiseLowerSpinner.encoder.position >= Constants.INTAKE_ARM_HOLD_THRESHOLD)
            intake.holdArm()
        else
            intake.stopArm()
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        intake.stopArm()
    }
}
