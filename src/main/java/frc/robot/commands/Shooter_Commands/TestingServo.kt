package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Shooter

class TestingServo(private var shooter: Shooter, private var angle: Double) : CommandBase() {

    init {
        addRequirements(shooter)
    }

    override fun initialize() {
        shooter.servo.angle = angle

    }
}