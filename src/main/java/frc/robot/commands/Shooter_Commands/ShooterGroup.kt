package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.Intake_Commands.IntakeCargos
import frc.robot.commands.Intake_Commands.IntakeGroup
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

/**
 * Used to run ShooterCargo
 * @param shooter: StaticShooter
 */
class ShooterGroup(intake: Intake, intakeVel: Double, shooter: Shooter, static: Boolean, velocity: ()->Double) : ParallelCommandGroup() {


    init {
        addCommands(
            ShooterCargos(shooter, static, velocity),
            IntakeCargos(intake, intakeVel)
        )

    }


}