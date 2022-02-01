package frc.robot.util

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import kotlin.math.abs

class SafeTalonFX (deviceNumber: Int) : WPI_TalonFX(deviceNumber) {

    private var usePID: Boolean = false
    private val currentLimit = 40
    private val currentThreshold = 0
    private val currentThresholdTime = 0.0 // In Milliseconds
    private val maxSpeed = 21000

    private val deadBand = 0.05

    constructor(deviceNumber: Int, usePID: Boolean) : this(deviceNumber) {
        this.usePID = usePID
    }

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