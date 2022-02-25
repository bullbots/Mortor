package frc.robot.commands.Climber_Commands

import com.ctre.phoenix.motorcontrol.TalonFXControlMode
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Climber
import kotlin.math.abs

class Grenade(private var climber: Climber) : CommandBase() {

    init {
        addRequirements(climber)
    }

    override fun initialize() {
        climber.climberMotor.set(TalonFXControlMode.MotionMagic, Constants.CLIMBER_DOWN)

    }

    override fun execute() {
    }

    override fun end(interrupted: Boolean) { climber.stop() }

    override fun isFinished(): Boolean {
        return abs(climber.climberMotor.getClosedLoopError(Constants.kPIDLoopIdx)) < 10

    }

}