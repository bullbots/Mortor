package frc.robot.commands.Shooter_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.util.function.DoubleConsumer;

public class ShooterGroup extends SequentialCommandGroup {

    /**
     * Used to run ShooterCargo
     * @param shooter: StaticShooter
     */
    public ShooterGroup(Intake intake, Shooter shooter, boolean static_var, DoubleConsumer velocity) {
        addCommands(
            new FeedCargo(intake, -0.3).withTimeout(0.04),
            new ShooterCargos(shooter, static_var, velocity).withTimeout(0.75),
            new ParallelCommandGroup(
                new FeedCargo(intake, 0.3),
                new ShooterCargos(shooter, static_var, velocity)
            )
        );
    }


}
