package frc.robot.commands.Drivetrain_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.DrivetrainFalcon

class DriveForTimeCommand(private val drivetrainFalcon: DrivetrainFalcon, time: Double) : WaitCommand(time) {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(drivetrainFalcon)
    }

    override fun initialize() {
        super.initialize()
        drivetrainFalcon.arcadeDrive(0.5, 0.0, true)
        println("INFO: DriveForTimeCommand initialize")
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        println("INFO: DriveForTimeCommand end")
    }
}
