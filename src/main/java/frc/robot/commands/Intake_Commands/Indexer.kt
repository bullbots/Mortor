package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Shooter

class Indexer(private val shooter: Shooter, private val shooterVel: Double) : CommandBase() {

    // each subsystem used by the command must be passed into the addRequirements() method
    init { addRequirements(shooter) }

    override fun initialize() { shooter.shooterSpinner.set(shooterVel) }

    override fun execute() {}


    override fun isFinished(): Boolean { return false }

    override fun end(interrupted: Boolean) { shooter.shooterSpinner.stopMotor() }
}
