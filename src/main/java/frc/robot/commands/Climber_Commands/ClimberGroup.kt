package frc.robot.commands.Climber_Commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.Climber

class ClimberGroup(climber: Climber, climberVel: Double = 0.1) : SequentialCommandGroup() {

    init {
        addCommands(
            ManualClimber(climber, climberVel)
        )
    }
}