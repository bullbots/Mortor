package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.Intake_Commands.IntakeCargos
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

/**
 * Used to run ShooterCargo
 * @param shooter: StaticShooter
 */
class ShooterGroup(intake: Intake, intakeVel: Double, shooter: Shooter, static: Boolean, velocity: ()->Double) : SequentialCommandGroup() {

    init {
        addCommands(
            IntakeCargos(intake, intakeVel, 0.0, shooter).withTimeout(0.04),
            ShooterCargos(shooter, static, velocity).withTimeout(0.75),
            ParallelCommandGroup(
                IntakeCargos(intake, 0.1, 0.0, shooter),
                ShooterCargos(shooter, static, velocity)
            )
        )
    }


}