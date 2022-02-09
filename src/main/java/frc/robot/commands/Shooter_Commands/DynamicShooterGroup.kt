package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.DynamicShooter

class DynamicShooterGroup(dynamicShooter: DynamicShooter) : SequentialCommandGroup() {

    init {
        addCommands(DynamicShooterCargo(dynamicShooter))
    }
}