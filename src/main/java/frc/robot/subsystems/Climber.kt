package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.wpilibj.Counter
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeTalonFX

class Climber : SubsystemBase() {

    var loopIdx = 0
    var climberMotor: SafeTalonFX
    var hallEffectTop: Counter
    var hallEffectBot: Counter

    var delta = 0.0

    var currentState = State.RELEASE
    var isReleased = false


    companion object{
        enum class State {
            BOTTOM, RELEASE, TOP
        }

//        var isReleased = false
    }

    init {
//        configureShuffleBoard()

        // Initializing Motor(s)
        climberMotor = SafeTalonFX(Constants.CLIMBER_PORT, isDrivetrain=false, usePID=false)

        climberMotor.setNeutralMode(NeutralMode.Brake)

        climberMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx)
        climberMotor.config_kF(Constants.kSlotIdx, Constants.climberkF, Constants.kTIMEOUT_MS)
        climberMotor.config_kP(Constants.kSlotIdx, Constants.climberkP, Constants.kTIMEOUT_MS)
        climberMotor.config_kI(Constants.kSlotIdx, Constants.climberkI, Constants.kTIMEOUT_MS)
        climberMotor.config_kD(Constants.kSlotIdx, Constants.climberkD, Constants.kTIMEOUT_MS)

        climberMotor.configMotionCruiseVelocity(15000.0, Constants.kTIMEOUT_MS)
        climberMotor.configMotionAcceleration(15000.0, Constants.kTIMEOUT_MS)


        hallEffectTop = Counter(Counter.Mode.kPulseLength)
        hallEffectBot = Counter(Counter.Mode.kPulseLength)

        hallEffectTop.setUpSource(0)
        hallEffectBot.setUpSource(1)

        hallEffectTop.setUpSourceEdge(true, true)
        hallEffectBot.setUpSourceEdge(true, true)

        hallEffectTop.reset()
        hallEffectBot.reset()
    }

    private fun configureShuffleBoard() {}

    override fun periodic() {
        loopIdx++
        if (loopIdx == 10) {
            loopIdx = 0
            SmartDashboard.putNumber("Climber PID Error", climberMotor.getClosedLoopError(Constants.kPIDLoopIdx))
            SmartDashboard.putNumber("Climber Velocity", climberMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx))
            SmartDashboard.putNumber("Climber Position", climberMotor.getSelectedSensorPosition(Constants.kPIDLoopIdx))
            SmartDashboard.putNumber("Climber Current", climberMotor.statorCurrent)
            SmartDashboard.putNumber("Climber Active Traj Pos", climberMotor.activeTrajectoryPosition)
        }

//        checkHallEffectSoftLimits()
    }

    fun stop() { climberMotor.set(0.0) }

    fun checkHallEffectSoftLimits() {
        // Used for the HallEffect Sensors
        if(currentState == State.RELEASE) {
            if (hallEffectTop.get() > 0) {
                currentState = State.TOP
                delta = climberMotor.selectedSensorPosition
            } else if (hallEffectBot.get() > 0) {
                currentState = State.BOTTOM
                delta = climberMotor.selectedSensorPosition
            }
        } else if (currentState == State.BOTTOM) {
            if(climberMotor.selectedSensorPosition - delta > Constants.CLIMBER_LIMIT_THRESHOLD) {
                currentState = State.RELEASE
                hallEffectBot.reset()
            }
        } else if (currentState == State.TOP) {
            if (delta - climberMotor.selectedSensorPosition > Constants.CLIMBER_LIMIT_THRESHOLD) {
                currentState = State.RELEASE
                hallEffectTop.reset()
            }
        }
    }

    fun setManual(percentOutput: Double) {
        if (climberMotor.selectedSensorPosition <= -14000) {
            climberMotor.stopMotor()
            println("WARNING: THE CLIMBER IS TO LOW!!!!!!")
        } else {
            climberMotor.set(percentOutput)
        }
    }
    fun encoderSoftLimits() {

    }
}