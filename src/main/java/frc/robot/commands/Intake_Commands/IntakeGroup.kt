package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.Shooter_Commands.ShooterCargos
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

/**
 * Used to run IntakeCargo
 * @param intake: Intake
 * @param intakeVel: Double / The default value is 0.3 if a value is not passed in
 */
class IntakeGroup(intake: Intake, intakeVel: Double, shooter: Shooter, velocity: ()->Double = { -0.1 }) : ParallelCommandGroup() {

    init {
        addCommands(
            AutoArmCommand(intake, isDown=true),
            IntakeCargos(intake, intakeVel).beforeStarting(WaitCommand(0.2)),
            ShooterCargos(shooter, true, velocity)
        )

    }
}