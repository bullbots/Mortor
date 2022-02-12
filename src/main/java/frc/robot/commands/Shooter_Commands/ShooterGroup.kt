package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.Shooter

/**
 * Used to run ShooterCargo
 * @param shooter: StaticShooter
 */
class ShooterGroup(shooter: Shooter, static: Boolean, velocity: ()->Double) : SequentialCommandGroup() {


    init {
        addCommands(ShooterCargos(shooter, static, velocity))

    }


}