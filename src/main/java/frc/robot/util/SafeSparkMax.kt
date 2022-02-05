package frc.robot.util

import com.revrobotics.CANSparkMax

class SafeSparkMax(deviceNumber: Int, motorType: MotorType = MotorType.kBrushless) : CANSparkMax(deviceNumber, motorType) {

    private val stallLimit = 40
    private val freeLimit = 40

    init {

        clearFaults()
        setSmartCurrentLimit(stallLimit, freeLimit)
        burnFlash()
        restoreFactoryDefaults()

    }
}