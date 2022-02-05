package frc.robot.util

import com.revrobotics.CANSparkMax


/**
 * Wrapper used to prevent NEOs from burning out or being damaged
 * @param deviceNumber Int
 * @param motorType MotorType: By default this value is kBrushless
 */
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