package frc.robot.commands.Drivetrain_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainFalcon

class Drive(private var drivetrain: DrivetrainFalcon, private var velocity: Double) : CommandBase() {

    init {
        addRequirements(drivetrain)
    }

    override fun initialize() {
        drivetrain.leftGroup.set(velocity)
        drivetrain.rightGroup.set(velocity)
    }

    override fun execute() {}

    override fun end(interrupted: Boolean) {
        drivetrain.stop()
    }

    override fun isFinished(): Boolean {
        return false
    }
}