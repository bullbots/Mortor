package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice
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

    companion object{
        enum class State {
            BOTTOM, RELEASE, TOP
        }
    }

    init {
        configureShuffleBoard()


        hallEffectTop = Counter(Counter.Mode.kPulseLength)
        hallEffectBot = Counter(Counter.Mode.kPulseLength)

        hallEffectTop.setUpSource(0)
        hallEffectBot.setUpSource(1)

        hallEffectTop.setUpSourceEdge(true, true)
        hallEffectBot.setUpSourceEdge(true, true)


        // Initializing Motors
        climberMotor = SafeTalonFX(Constants.CLIMBER_PORT)
        climberMotor.configFactoryDefault()

        climberMotor.setNeutralMode(NeutralMode.Brake)

        climberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
            Constants.kTIMEOUT_MS)

        climberMotor.configNeutralDeadband(0.01, Constants.kTIMEOUT_MS)

        climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTIMEOUT_MS)
        climberMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, Constants.kTIMEOUT_MS)

        climberMotor.configNominalOutputForward(0.0, Constants.kTIMEOUT_MS)
        climberMotor.configNominalOutputReverse(0.0, Constants.kTIMEOUT_MS)
        climberMotor.configPeakOutputForward(1.0, Constants.kTIMEOUT_MS)
        climberMotor.configPeakOutputReverse(-1.0, Constants.kTIMEOUT_MS)

        climberMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx)
        climberMotor.config_kF(Constants.kSlotIdx, Constants.climberkF, Constants.kTIMEOUT_MS)
        climberMotor.config_kP(Constants.kSlotIdx, Constants.climberkP, Constants.kTIMEOUT_MS)
        climberMotor.config_kI(Constants.kSlotIdx, Constants.climberkI, Constants.kTIMEOUT_MS)
        climberMotor.config_kD(Constants.kSlotIdx, Constants.climberkD, Constants.kTIMEOUT_MS)

        climberMotor.configMotionCruiseVelocity(15000.0, Constants.kTIMEOUT_MS)
        climberMotor.configMotionAcceleration(6000.0, Constants.kTIMEOUT_MS)

        climberMotor.setSelectedSensorPosition(0.0, Constants.kPIDLoopIdx, Constants.kTIMEOUT_MS)




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
        }



        // Used for the HallEffect Sensors
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
    }




    fun stop() { climberMotor.set(0.0) }


}