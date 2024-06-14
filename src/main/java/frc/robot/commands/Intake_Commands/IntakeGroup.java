package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.commands.Shooter_Commands.ShooterCargos;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeGroup extends ParallelCommandGroup {

    /**
     * Used to run IntakeCargo
     * @param intake: Intake
     * @param intakeVel: Double / The default value is 0.3 if a value is not passed in
     */
    public IntakeGroup(Intake intake, double intakeVel, double armVel, Shooter shooter) {
        addCommands(
            new IntakeCargos(intake, intakeVel, armVel, shooter),
            new Indexer(shooter, -0.3)
        );

    }
}
