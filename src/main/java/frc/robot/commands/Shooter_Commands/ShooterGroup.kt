package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

/**
 * Used to run ShooterCargo
 * @param shooter: StaticShooter
 */
class ShooterGroup(intake: Intake, shooter: Shooter, static: Boolean, velocity: ()->Double) : SequentialCommandGroup() {

    init {
        addCommands(
            FeedCargo(intake, -0.3).withTimeout(0.04),
            ShooterCargos(shooter, static, velocity).withTimeout(0.75),
            ParallelCommandGroup(
                FeedCargo(intake, 0.3),
                ShooterCargos(shooter, static, velocity)
            )
        )
    }


}