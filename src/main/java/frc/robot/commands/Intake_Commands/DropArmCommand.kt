package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Intake

class DropArmCommand(private val intake: Intake, private var armVel: Double = 0.5) : CommandBase() {
    var allowDrop = false

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intake)
    }

    override fun initialize() {
        allowDrop = false
    }

    override fun execute() {
//        if (intake.intakeSpinner.encoder.position < Constants.INTAKE_ARM_DROP_THRESHOLD && !allowDrop) {
//            intake.intakeSpinner.set(armVel)
//            println("INFO: set armvel: $armVel")
//        } else {
//            allowDrop = true
//            intake.intakeSpinner.stopMotor()
//            println("INFO: dropping arm")
//        }
        intake.armSpinner.set(0.3)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        println("INFO: DropArmCommand: end")
        intake.armSpinner.stopMotor()
    }
}
