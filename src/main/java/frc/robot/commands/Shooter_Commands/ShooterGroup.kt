package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.StaticShooter

/**
 * Used to run ShooterCargo
 * @param staticShooter: StaticShooter
 */
class ShooterGroup(staticShooter: StaticShooter) : SequentialCommandGroup() {


    init {
        addCommands(ShooterCargos(staticShooter))

    }


}