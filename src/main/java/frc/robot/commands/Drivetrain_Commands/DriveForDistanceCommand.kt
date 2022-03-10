package frc.robot.commands.Drivetrain_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainFalcon

class DriveForDistanceCommand(private val drivetrainFalcon: DrivetrainFalcon,
                              private val speed: Double,
                              private val distance: Double) : CommandBase() {

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(drivetrainFalcon)
    }

    override fun initialize() {
        drivetrainFalcon.arcadeDrive(speed, 0.0, squareInputs=false)
        println("INFO: DriveForDistanceCommand initialize")
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        val averageDist = drivetrainFalcon.getAverageDist()
        if (averageDist >= distance) {
            return true
        }
        return false
    }

    override fun end(interrupted: Boolean) {
        drivetrainFalcon.stop()
        println("INFO: DriveForDistanceCommand end")
    }
}
