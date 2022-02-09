package frc.robot.util

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import kotlin.math.abs

/**
 * Wrapper used to prevent Falcon 500 from burning out or being damaged
 * @param deviceNumber Int
 * @param usePID Boolean: By default this value is false.
 */
class SafeTalonFX (deviceNumber: Int, private var usePID: Boolean = false) : WPI_TalonFX(deviceNumber) {

    private val currentLimit = 40
    private val currentThreshold = 0
    private val currentThresholdTime = 0.0 // In Milliseconds
    private val maxSpeed = 21000

    private val deadBand = 0.05

    init {
        configFactoryDefault()
        val config = StatorCurrentLimitConfiguration(
            true,
            currentLimit.toDouble(),
            currentThreshold.toDouble(),
            currentThresholdTime
        )
        configNeutralDeadband(deadBand)

        // configStatorCurrentLimit(config);
        configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30)
        configNominalOutputForward(0.0, 30)
        configNominalOutputReverse(0.0, 30)
        configPeakOutputForward(1.0, 30)
        configPeakOutputReverse(-1.0, 30)
    }

    /**
     * Moves the motor at a percent of its max speed
     * @param percentOutput A double from -1 to 1
     */
    override fun set(percentOutput: Double) {
        if (usePID && abs(percentOutput) > 0.1) {
            super.set(ControlMode.Velocity, percentOutput * maxSpeed)
        } else {
            super.set(percentOutput)
        }
    }
}