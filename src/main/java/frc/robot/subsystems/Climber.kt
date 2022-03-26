package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.TalonFXControlMode
import edu.wpi.first.wpilibj.Counter
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeTalonFX

class Climber : SubsystemBase() {

    var loopIdx = 0
    var climberMotor: SafeTalonFX
//    var hallEffectTop: Counter
//    var hallEffectBot: Counter
    var limitSwitch: Counter

    var delta = 0.0

    var currentState = State.RELEASE



    companion object{
        enum class State {
            BOTTOM, RELEASE, TOP
        }

        var isReleased = false
    }

    init {
//        configureShuffleBoard()

        // Initializing Motor(s)
        climberMotor = SafeTalonFX(Constants.CLIMBER_PORT, isDrivetrain=false, usePID=false)

        climberMotor.setNeutralMode(NeutralMode.Brake)

        climberMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx)
        climberMotor.config_kF(Constants.kSlotIdx, Constants.CLIMBER_KFF, Constants.kTIMEOUT_MS)
        climberMotor.config_kP(Constants.kSlotIdx, Constants.CLIMBER_KP, Constants.kTIMEOUT_MS)
        climberMotor.config_kI(Constants.kSlotIdx, Constants.CLIMBER_KI, Constants.kTIMEOUT_MS)
        climberMotor.config_kD(Constants.kSlotIdx, Constants.CLIMBER_KD, Constants.kTIMEOUT_MS)

        climberMotor.configMotionCruiseVelocity(21000.0, Constants.kTIMEOUT_MS)
        climberMotor.configMotionAcceleration(21000.0, Constants.kTIMEOUT_MS)


        limitSwitch = Counter(Counter.Mode.kPulseLength)

        limitSwitch.setUpSource(0)
//        hallEffectTop = Counter(Counter.Mode.kPulseLength)
//        hallEffectBot = Counter(Counter.Mode.kPulseLength)
//
//        hallEffectTop.setUpSource(2)
//        hallEffectBot.setUpSource(1)
//
//        hallEffectTop.setUpSourceEdge(true, true)
//        hallEffectBot.setUpSourceEdge(true, true)
//
//        hallEffectTop.reset()
//        hallEffectBot.reset()
    }

    private fun configureShuffleBoard() {}



    override fun periodic() {
        loopIdx++
        if (loopIdx == 10) {
            loopIdx = 0
//            SmartDashboard.putNumber("Climber PID Error", climberMotor.getClosedLoopError(Constants.kPIDLoopIdx))
//            SmartDashboard.putNumber("Climber Velocity", climberMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx))
            SmartDashboard.putNumber("Climber Position", climberMotor.getSelectedSensorPosition(Constants.kPIDLoopIdx))
            SmartDashboard.putNumber("Climber Supply Current", climberMotor.supplyCurrent)
            SmartDashboard.putNumber("Limit Switch", limitSwitch.get().toDouble())
//            SmartDashboard.putNumber("Climber Active Traj Pos", climberMotor.activeTrajectoryPosition)
        }

//        checkHallEffectSoftLimits()
    }

    fun stop() { climberMotor.set(0.0) }

//    fun checkHallEffectSoftLimits() {
//        // Used for the HallEffect Sensors
//        if(currentState == State.RELEASE) {
//            if (hallEffectTop.get() > 0) {
//                currentState = State.TOP
//                delta = climberMotor.selectedSensorPosition
//            } else if (hallEffectBot.get() > 0) {
//                currentState = State.BOTTOM
//                delta = climberMotor.selectedSensorPosition
//            }
//        } else if (currentState == State.BOTTOM) {
//            if(climberMotor.selectedSensorPosition - delta > Constants.CLIMBER_LIMIT_THRESHOLD) {
//                currentState = State.RELEASE
//                hallEffectBot.reset()
//            }
//        } else if (currentState == State.TOP) {
//            if (delta - climberMotor.selectedSensorPosition > Constants.CLIMBER_LIMIT_THRESHOLD) {
//                currentState = State.RELEASE
//                hallEffectTop.reset()
//            }
//        }
//    }
    fun set(encoderVal: Double, controlMode: TalonFXControlMode) {
        if (limitSwitch.get() > 0) {
            if (encoderVal > 0) {
                climberMotor.set(controlMode, encoderVal)
                limitSwitch.reset()
            } else {
                println("WARNING: The climber is too low!!!!!!")
                climberMotor.stopMotor()
            }
        } else {
            climberMotor.set(controlMode, encoderVal)
        }
    }

    fun setManual(percentOutput: Double) {
        if (limitSwitch.get() > 0) {
            if(percentOutput < 0) {
                climberMotor.stopMotor()
                println("WARNING: THE CLIMBER IS TO LOW!!!!!!")
            } else {
                climberMotor.set(percentOutput)
                limitSwitch.reset()
            }
        } else {
            climberMotor.set(percentOutput)
        }
    }
    fun encoderSoftLimits() {

    }
}