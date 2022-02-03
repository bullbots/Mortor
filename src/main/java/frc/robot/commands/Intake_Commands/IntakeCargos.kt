package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.CommandBase

import frc.robot.subsystems.Intake

class IntakeCargos(intake: Intake) : CommandBase() {
    /**
     * Creates a new IntakeCargo
     */
    private var intake: Intake? = null

    // Use addRequirements() here to declare subsystem dependencies.
    init {
        addRequirements(intake)
        this.intake = intake
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        intake?.setIntake()
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {}

    override fun end(interrupted: Boolean) {
        intake?.stop()
    }

    override fun isFinished(): Boolean {
        return false
    }

}