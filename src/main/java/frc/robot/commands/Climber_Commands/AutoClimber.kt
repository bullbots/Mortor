package frc.robot.commands.Climber_Commands

import com.ctre.phoenix.motorcontrol.TalonFXControlMode
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Climber
import kotlin.math.abs

class AutoClimber(private var climber: Climber, private var isGrenade: Boolean, private var isDown: Boolean) : CommandBase() {
    private var targetTraj = 0.0
    private var isReleased: Boolean

    init { addRequirements(climber)
        isReleased = false
        if (isGrenade) {
            targetTraj = Constants.CLIMBER_DOWN
            isReleased = true
        } else if(isReleased) {
            targetTraj = if (isDown) {
                Constants.CLIMBER_DOWN
            } else {
                Constants.CLIMBER_UP
            }
        } else {
            println("PULL GRENADE PIN")
        }
    }

    override fun initialize() {
        println("INFO: Grenade initialize")
        climber.climberMotor.set(TalonFXControlMode.MotionMagic, targetTraj)
    }

    override fun execute() {
//        println("INFO: Climber Executed")
    }

    override fun isFinished(): Boolean {
        val traj_pose_error = targetTraj - climber.climberMotor.getSelectedSensorPosition(Constants.kPIDLoopIdx)
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