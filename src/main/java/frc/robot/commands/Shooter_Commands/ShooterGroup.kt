package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.Intake_Commands.IntakeCargos
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Intake

/**
 * Used to run ShooterCargo
 * @param shooter: StaticShooter
 */
class ShooterGroup(shooter: Shooter, static: Boolean, intake: Intake, velocity: ()->Double) : SequentialCommandGroup() {

    init {
        addCommands(
                IntakeCargos(intake, -0.1).withTimeout(0.05),
                ShooterCargos(shooter, static, velocity).withTimeout(1.3),
                ParallelCommandGroup(
                        IntakeCargos(intake, 0.1),
                        ShooterCargos(shooter, static, velocity)
                )
        )
    }
}