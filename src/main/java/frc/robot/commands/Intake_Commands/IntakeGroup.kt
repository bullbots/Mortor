package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.Intake

/**
 * Used to run IntakeCargo
 * @param intake: Intake
 * @param intakeVel: Double / The default value is 0.3 if a value is not passed in
 */
class IntakeGroup(intake: Intake, intakeVel: Double = 0.3) : SequentialCommandGroup() {

    init {
        addCommands(IntakeCargos(intake, intakeVel))
    }
}