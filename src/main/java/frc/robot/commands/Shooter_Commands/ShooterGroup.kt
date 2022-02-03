package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.StaticShooter

class ShooterGroup(staticShooter: StaticShooter) : SequentialCommandGroup() {

    init {
        addCommands(ShooterCargos(staticShooter))
    }
}