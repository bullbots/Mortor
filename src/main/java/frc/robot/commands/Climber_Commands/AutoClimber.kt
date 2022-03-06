package frc.robot.commands.Climber_Commands

import com.ctre.phoenix.motorcontrol.TalonFXControlMode
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Climber
import kotlin.math.abs

class AutoClimber(private var climber: Climber, private var isGrenade: Boolean, private var isDown: Boolean) : CommandBase() {
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
            climber.climberMotor.selectedSensorPosition = 0.0
            println("INFO: Grenade initialize")
        } else if (!Climber.isReleased) {
            println("PULL GRENADE PIN!!!!!!!!!!")
            return
        }

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