package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.Shooter_Commands.ShooterCargos
import frc.robot.subsystems.Intake
import frc.robot.subsystems.StaticShooter

/**
 * Used to run IntakeCargo
 * @param intake: Intake
 * @param intakeVel: Double / The default value is 0.3 if a value is not passed in
 */
class IntakeGroup(intake: Intake, intakeVel: Double, staticShooter: StaticShooter, velocity: ()->Double = { 0.05 }) : SequentialCommandGroup() {

    init {
        addCommands(
            IntakeCargos(intake, intakeVel),
            ShooterCargos(staticShooter, velocity)
        )

    }
}