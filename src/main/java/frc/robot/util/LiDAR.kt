package frc.robot.util

import edu.wpi.first.wpilibj.Counter
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class LiDAR : TimedRobot() {

    lateinit var m_LiDAR: Counter

    override fun robotInit() {
        m_LiDAR = Counter(0)
        m_LiDAR.setMaxPeriod(1.00)
        m_LiDAR.setSemiPeriodMode(true)
        m_LiDAR.reset()
    }

    val off = 10.0

    override fun robotPeriodic() {
        var dist: Double
        if(m_LiDAR.get() < 1) {
            dist = 0.0
        } else {
            dist = (m_LiDAR.period * 1000000.0 / 10.0) - off
            SmartDashboard.putNumber("LiDAR_Distance", dist)
        }
    }

}