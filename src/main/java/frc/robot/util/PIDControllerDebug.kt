// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import java.lang.reflect.Field
import kotlin.math.abs

/**
 * Wrapper for the PIDController setting the PID values
 * @param m_kp: Double
 * @param m_ki: Double
 * @param kd: Double
 */
class PIDControllerDebug(private val m_kp: Double, private val m_ki: Double, kd: Double) :
    PIDController(m_kp, m_ki, kd) {
    lateinit var totalErrorField: Field
    private var prev = false

    init {
        try {
            totalErrorField = PIDController::class.java.getDeclaredField("m_totalError")
            totalErrorField.isAccessible = true
        } catch (e: NoSuchFieldException) {
            e.printStackTrace()
        } catch (e: SecurityException) {
            e.printStackTrace()
        }
//        SmartDashboard.putNumber("PID Position Output", 0.0)
//        SmartDashboard.putNumber("PID Integral Output", 0.0)
//        SmartDashboard.putNumber("PID output", 0.0)
    }

//    override fun calculate(measurement: Double): Double {
//        var output = 0.0
//        var outputRegion = 0
//        if (abs(measurement) <= Constants.VISION_OUTER_ALIGN_THRESHOLD) {
//            if (!prev) {
//                println("INFO: Calling PID reset")
//                reset()
//            }
//            output = super.calculate(measurement)
//            prev = true
//            outputRegion = 1
//        } else {
//            prev = false
//        }
//        SmartDashboard.putNumber("PID Region", outputRegion.toDouble())
//
//        /*
//        // m_measurement = measurement;
//        // m_prevError = m_positionError;
//        // if (m_continuous) {
//        // m_positionError =
//        // MathUtil.inputModulus(m_setpoint - measurement, m_minimumInput,
//        // m_maximumInput);
//        // } else {
//        // m_positionError = m_setpoint - measurement;
//        // }
//        // m_velocityError = (m_positionError - m_prevError) / m_period;
//        // if (m_ki != 0) {
//        // m_totalError =
//        // MathUtil.clamp(
//        // m_totalError + m_positionError * m_period,
//        // m_minimumIntegral / m_ki,
//        // m_maximumIntegral / m_ki);
//        // }
//        // return m_kp * m_positionError + m_ki * m_totalError + m_kd * m_velocityError;
//        */
//        var totalError = 0.0
//        try {
//            totalError = totalErrorField[this] as Double
//        } catch (e: IllegalArgumentException) {
//            e.printStackTrace()
//        } catch (e: IllegalAccessException) {
//            e.printStackTrace()
//        }
//        val positionOutput = m_kp * positionError
//        val integralOutput = m_ki * totalError
//        SmartDashboard.putNumber("PID Position Output", positionOutput)
//        SmartDashboard.putNumber("PID Integral Output", integralOutput)
//        SmartDashboard.putNumber("PID output", output)
//        return output
//    }
}