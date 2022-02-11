package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.StaticShooter

/**
 * Used to run ShooterCargo
 * @param staticShooter: StaticShooter
 */
class ShooterGroup(staticShooter: StaticShooter, velocity: ()->Double) : SequentialCommandGroup() {


    init {
        addCommands(ShooterCargos(staticShooter, velocity))

    }


}