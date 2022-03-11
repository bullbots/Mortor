package frc.robot.commands.Shooter_Commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Shooter

/**
 * ShooterCargo initialize the velocity of the staticShooter
 * @param shooter: StaticShooter
 */
class ShooterCargos(private var shooter: Shooter, private var static: Boolean, private var dist: () -> Double) : CommandBase() {

    var velocity = 0.0
    // Use addRequirements() here to declare subsystem dependencies.
    init {
        addRequirements(shooter)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        // var shooterVel = SmartDashboard.getNumber("StaticShooter", 0.3)
        if(static) {
            velocity = dist()
            shooter.shooterSpinner.set(dist())
        } else {
            // dist() is in feet
            velocity = when (dist()) {
                in 1.0..6.0 -> {
                    println("This value is between 1.0 and 6.0")
                    0.0
                }
                in 6.0..7.0 -> {
                    println("This value is between 6.0 and 7.0")
                    0.0
                }
                in 10.0..12.0 -> MathUtil.interpolate(0.46, 0.50, (dist()-10 / 2)) // Template
                in 14.0..16.0 -> MathUtil.interpolate(0.50, 0.56, (dist()-14) / 2)
                else -> 0.0
            }
        }
//        println(velocity) // Debugging Values
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {}

    override fun end(interrupted: Boolean) { shooter.stop() }

    override fun isFinished(): Boolean { return false }
}