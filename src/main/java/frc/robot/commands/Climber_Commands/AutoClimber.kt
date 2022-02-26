package frc.robot.commands.Climber_Commands

import com.ctre.phoenix.motorcontrol.TalonFXControlMode
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Climber
import kotlin.math.abs

class AutoClimber(private var climber: Climber, private var isGrenade: Boolean, private var isDown: Boolean) : CommandBase() {

    init { addRequirements(climber) }

    override fun initialize() {
        println("INFO: Grenade initialize")
        if(isGrenade) {
            climber.climberMotor.set(TalonFXControlMode.MotionMagic, Constants.CLIMBER_DOWN)
        } else if (!isGrenade) {
            if (isDown) {
                climber.climberMotor.set(TalonFXControlMode.MotionMagic, Constants.CLIMBER_DOWN)
            } else if (!isDown) {
                climber.climberMotor.set(TalonFXControlMode.MotionMagic, Constants.CLIMBER_UP)
            }
        }
    }

    override fun execute() {
        println("INFO: Climber Executed")
    }

    override fun isFinished(): Boolean {
        return if (isGrenade) {
            abs(climber.climberMotor.getClosedLoopError(Constants.kPIDLoopIdx)) < 100
        } else {
            false
        }
    }

    override fun end(interrupted: Boolean) {
        println("INFO: Grenade end")
        climber.stop()
    }
}