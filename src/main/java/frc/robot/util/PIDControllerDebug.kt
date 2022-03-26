// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import java.lang.reflect.Field
import kotlin.math.abs
import kotlin.math.sign

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

//    fun totalError(): Double {
//        var totalError = 0.0
//        try {
//            totalError = totalErrorField[this] as Double
//        } catch (e: IllegalArgumentException) {
//            e.printStackTrace()
//        } catch (e: IllegalAccessException) {
//            e.printStackTrace()
//        }
//        return totalError
//    }

    init {
        try {
            totalErrorField = PIDController::class.java.getDeclaredField("m_totalError")
            totalErrorField.isAccessible = true
        } catch (e: NoSuchFieldException) {
            e.printStackTrace()
        } catch (e: SecurityException) {
            e.printStackTrace()
        }
//        SmartDashboard.putNumber("PID Position Output", positionError * p)
//        SmartDashboard.putNumber("PID Integral Output", totalErrorField.getDouble(totalErrorField) * i)
//        SmartDashboard.putNumber("PID output", 0.0)
    }

    fun calculateDebug(measurement: Double, setpoint: Double, displayOutput: Boolean): Double {
        var outputRegion = 0
        val delta = setpoint - measurement

        var output = if (abs(delta) > 45) {
            sign(delta) * 0.55
            0.0
        } else if(abs(delta) > 1){
            sign(delta) * 0.35
        } else {
            0.0
        }

        if (displayOutput) {
            var totalError = 0.0
            try {
                totalError = totalErrorField[this] as Double
            } catch (e: IllegalArgumentException) {
                e.printStackTrace()
            } catch (e: IllegalAccessException) {
                e.printStackTrace()
            }
            val positionOutput = m_kp * positionError
            val integralOutput = m_ki * totalError

            SmartDashboard.putNumber("PID Position Output", positionOutput)
            SmartDashboard.putNumber("PID Integral Output", integralOutput)
//        SmartDashboard.putNumber("PID Derivative Output", )
            SmartDashboard.putNumber("PID output", output)
            SmartDashboard.putNumber("PID Expected Output", positionOutput + integralOutput + (sign(delta) * 0.1))


        }

        return output

    }




}