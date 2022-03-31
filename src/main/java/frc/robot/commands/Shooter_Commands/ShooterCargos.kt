package frc.robot.commands.Shooter_Commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Shooter

/**
 * ShooterCargo initialize the velocity of the staticShooter
 * @param shooter: StaticShooter
 */
class ShooterCargos(private val shooter: Shooter, private val static: Boolean, private val dist: () -> Double) : CommandBase() {

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
                in 0.0..5.0 -> 0.26
                in 5.0..7.0 -> MathUtil.interpolate(0.38, 0.41, (dist()-5) / 2)
                in 7.0..9.0 -> MathUtil.interpolate(0.40, 0.43, (dist()-7) / 2)
                in 9.0..11.0 -> MathUtil.interpolate(0.43, 0.45, (dist()-9) / 2)
                in 11.0..13.0 -> MathUtil.interpolate(0.44, 0.46, (dist()-11 / 2))
                in 13.0..15.0 -> MathUtil.interpolate(0.46, 0.52, (dist()-13 / 2))
                in 15.0..17.0 -> MathUtil.interpolate(0.52, 0.58, (dist()-15) / 2)
                in 17.0..19.0 -> MathUtil.interpolate(0.57, 0.65, (dist()-17) / 2)
                in 19.0..21.0 -> MathUtil.interpolate(0.64, 0.7, (dist()-19) / 2)
                in 21.0..23.0 -> MathUtil.interpolate(0.69, 0.75, (dist()-21) / 2)
                in 23.0..26.0 -> MathUtil.interpolate(0.74, 0.82, (dist()-23) / 2)
                else -> 0.0
            }
            shooter.shooterSpinner.set(velocity)
        }
//        println(velocity) // Debugging Values
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {}

    override fun end(interrupted: Boolean) { shooter.stop() }

    override fun isFinished(): Boolean { return false }
}