package frc.robot.commands.Autonomous_Commands

import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.commands.Intake_Commands.IntakeGroup
import frc.robot.subsystems.DrivetrainFalcon
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import java.util.function.BooleanSupplier

class AutonomousGSC(drivetrain: DrivetrainFalcon, intake: Intake, shooter: Shooter, isLoaded: BooleanSupplier, isRed: BooleanSupplier, isA: BooleanSupplier) : SequentialCommandGroup() {

    init {
        addCommands(
            WaitUntilCommand(isLoaded),
            ParallelCommandGroup(
                ConditionalCommand(
                    TrajectoryBase(drivetrain, "/RED-COMBINED", isBackwards=true, resetGyro=true),
                    TrajectoryBase(drivetrain, "/BLUE-COMBINED", isBackwards=true, resetGyro=true),
                    isRed
                )
            ),
            IntakeGroup(intake, 0.0, shooter)
        )
    }


}