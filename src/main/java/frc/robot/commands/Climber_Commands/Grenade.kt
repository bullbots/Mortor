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
        println("INFO: Grenade initialize")
        climber.climberMotor.set(TalonFXControlMode.MotionMagic, Constants.CLIMBER_DOWN)
    }

    override fun execute() {
        println("INFO: Grenade execute")
    }

    override fun isFinished(): Boolean {
        return abs(climber.climberMotor.getClosedLoopError(Constants.kPIDLoopIdx)) < 100
    }

    override fun end(interrupted: Boolean) {
        println("INFO: Grenade end")
        climber.stop()
    }
}