package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.StaticShooter

class ShooterGroup(staticShooter: StaticShooter, shooterVel: Double) : SequentialCommandGroup() {


    init {
        addCommands(ShooterCargos(staticShooter, shooterVel))
        println("ShooterGroup: $shooterVel")
    }


}