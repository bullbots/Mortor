package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DynamicShooter
import frc.robot.util.LiDAR

class DynamicShooterCargo(private var dynamicShooter: DynamicShooter) : CommandBase() {

    var lidar = LiDAR()
    // Use addRequirements() here to declare subsystem dependencies.
    init {
        addRequirements(dynamicShooter)
        this.dynamicShooter = dynamicShooter
    }

    // Called when the command i sinitially scheduled.
    override fun initialize() {
        var dynamicShooterVel = lidar.getDist()
        // dynamicShooter.shooterSpinner.set(dynamicShooterVel)
        println(dynamicShooterVel)
    }
}