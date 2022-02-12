package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Shooter

/**
 * ShooterCargo initialize the velocity of the staticShooter
 * @param shooter: StaticShooter
 */
class ShooterCargos(private var shooter: Shooter, private var static: Boolean, private var velocity: () -> Double) : CommandBase() {

    // Use addRequirements() here to declare subsystem dependencies.
    init {
        addRequirements(shooter)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        // var shooterVel = SmartDashboard.getNumber("staticChooser", 0.3)
        if(static) {
            shooter.shooterSpinner.set(velocity())
        } else {
            when (velocity()) {
                2.0 -> println("This is 2")
                in 1.0..5.0 -> println("This value is between 1.0 and 5.0")
                in 6.0..100.0 -> println("This value is between 6.0 and 100.0")
            }
        }
        shooter.shooterSpinner.set(velocity())
        println(velocity())


    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {}

    override fun end(interrupted: Boolean) { shooter.stop() }

    override fun isFinished(): Boolean { return false }


}