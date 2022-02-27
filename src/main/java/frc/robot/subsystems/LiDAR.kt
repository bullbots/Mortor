package frc.robot.subsystems

import edu.wpi.first.wpilibj.Counter
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class LiDAR {

    private var m_LiDAR: Counter = Counter(0)

    private val off = 10.0
    var dist: Double = 0.0
        private set
        get() = if (m_LiDAR.get() < 1) 0.0 else (m_LiDAR.period * 100000.0) - off

    init {
        m_LiDAR.setMaxPeriod(1.00)
        m_LiDAR.setSemiPeriodMode(true)
        m_LiDAR.reset()
    }

}