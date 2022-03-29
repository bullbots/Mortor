package frc.robot.commands.Climber_Commands

import com.ctre.phoenix.motorcontrol.TalonFXControlMode
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Climber
import kotlin.math.abs

class AutoClimber(private val climber: Climber, private val isGrenade: Boolean, isDown: Boolean) : CommandBase() {
    private var targetTraj = 0.0

    init { addRequirements(climber)
        targetTraj = if(!isGrenade) {
            if (isDown) {
                Constants.CLIMBER_DOWN
            } else {
                Constants.CLIMBER_UP
            }
        } else {
            Constants.CLIMBER_GRENADE
        }
    }

    override fun initialize() {
        if (isGrenade) {
            Climber.isReleased = true
            climber.resetEncoders()
            println("INFO: Grenade initialize")
        } else if (!Climber.isReleased) {
            println("WARNING: PULL GRENADE PIN!!!!!!!!!!")
            return
        }
    }

    override fun execute() {
        climber.setAuto(TalonFXControlMode.MotionMagic, targetTraj)
    }

    override fun isFinished(): Boolean {
        val traj_pose_error = targetTraj - climber.getEncoderPos()
        return if (isGrenade) {
            abs(traj_pose_error) < 1000
        } else {
            false
        }
    }

    override fun end(interrupted: Boolean) {
        println("INFO: Grenade end")
        climber.stop()
    }

}
