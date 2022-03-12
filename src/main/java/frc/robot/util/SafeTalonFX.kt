package frc.robot.util

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import frc.robot.Constants
import kotlin.math.abs

/**
 * Wrapper used to prevent Falcon 500 from burning out or being damaged
 * @param deviceNumber Int
 * @param usePID Boolean: By default this value is false.
 * @param isDrivetrain Boolean: By default this value is true.
 */
class SafeTalonFX (deviceNumber: Int, private var isDrivetrain: Boolean = false, private var usePID: Boolean = false) : WPI_TalonFX(deviceNumber) {

    private val maxSpeed = 21000.0
    private val deadBand = 0.02

    init {
        configFactoryDefault()

        if (isDrivetrain) {
//            configStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 20.0, 25.0, 1.0))
//            configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 10.0, 15.0,0.5))
//            configStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40.0, 25.0, 1.0))
//            configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 20.0, 15.0,0.5))
        } else {
            configStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 70.0, 45.0, 1.0))
            configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 50.0, 30.0,0.5))
        }

        configNeutralDeadband(deadBand)

        setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTIMEOUT_MS)
        setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, Constants.kTIMEOUT_MS)

        // configStatorCurrentLimit(config);
        configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30)
        configNominalOutputForward(0.0, 30)
        configNominalOutputReverse(0.0, 30)
        configPeakOutputForward(1.0, 30)
        configPeakOutputReverse(-1.0, 30)
        setSelectedSensorPosition(0.0, Constants.kPIDLoopIdx, Constants.kTIMEOUT_MS)
    }

    /**
     * Moves the motor at a percent of its max speed
     * @param percentOutput A double from -1 to 1
     */
    override fun set(percentOutput: Double) {
        if (usePID && abs(percentOutput) > 0.02) {
//        if (usePID) {
            super.set(ControlMode.Velocity, percentOutput * maxSpeed)
//            println("INFO: velocity: ${percentOutput * maxSpeed}")
        } else {
            super.set(percentOutput)
        }
    }
}