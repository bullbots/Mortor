package frc.robot.util

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

/**
 * Used to find the angle of the robot in its current position
 * to keep orientated
 */
class NavX : AHRS() {

    private var angleDelta = 0.0

    init { angleDelta = angle }

    override fun getAngle(): Double {
        var angle = super.getAngle() - angleDelta
        angle = MathUtil.inputModulus(angle, -180.0, 180.0)

        return angle
    }

//    override fun reset() { angleDelta = super.getAngle() }

}