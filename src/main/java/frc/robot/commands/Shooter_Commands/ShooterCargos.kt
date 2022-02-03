package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.StaticShooter

class ShooterCargos(staticShooter: StaticShooter) : CommandBase() {
    /**
     * Creates a new IntakeCargo
     */
    private var staticShooter: StaticShooter? = null

    // Use addRequirements() here to declare subsystem dependencies.
    init {
        addRequirements(staticShooter)
        this.staticShooter = staticShooter
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        staticShooter?.setShooter()
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {}

    override fun end(interrupted: Boolean) {
        staticShooter?.stop()
    }

    override fun isFinished(): Boolean {
        return false
    }


}