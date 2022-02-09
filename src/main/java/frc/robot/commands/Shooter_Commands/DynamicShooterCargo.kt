package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DynamicShooter

class DynamicShooterCargo(private var dynamicShooter: DynamicShooter) : CommandBase() {

    // Use addRequirements() here to declare subsystem dependencies.
    init {
        addRequirements(dynamicShooter)
        this.dynamicShooter = dynamicShooter
    }

    // Called when the command i sinitially scheduled.
    override fun initialize() {
        var dynam
    }
}