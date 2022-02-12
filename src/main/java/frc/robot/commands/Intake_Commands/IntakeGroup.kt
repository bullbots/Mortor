package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.robot.commands.Shooter_Commands.ShooterCargos
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

/**
 * Used to run IntakeCargo
 * @param intake: Intake
 * @param intakeVel: Double / The default value is 0.3 if a value is not passed in
 */
class IntakeGroup(intake: Intake, intakeVel: Double, shooter: Shooter, velocity: ()->Double = { 0.0 }) : ParallelCommandGroup() {

    init {
        addCommands(
            IntakeCargos(intake, intakeVel),
            ShooterCargos(shooter, true, velocity)
        )

    }
}