package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.Intake

class IntakeGroup(intake: Intake, intakeVel: Double = 0.3) : SequentialCommandGroup() {

    init {
        addCommands(IntakeCargos(intake, intakeVel))
    }
}