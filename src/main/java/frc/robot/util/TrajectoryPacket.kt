package frc.robot.util

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Filesystem
import java.io.BufferedReader
import java.io.FileNotFoundException
import java.io.FileReader
import java.io.IOException
import kotlin.math.atan2


class TrajectoryPacket {

    var firstX = 0.0
    var firstY = 0.0

    lateinit var pathRead: ArrayList<Translation2d>

    var lastX = 0.0
    var lastY = 0.0

    var startAngle = 0.0
    var endAngle = 0.0

    companion object {

        fun generateTrajectoryPacket(pathname: String): TrajectoryPacket {
            var pathRead = ArrayList<Translation2d>()
            var angleList = ArrayList<Double>()

            var br: BufferedReader? = null

            try {
                br = BufferedReader(FileReader(Filesystem.getDeployDirectory().toString() + pathname))
            } catch (e: FileNotFoundException) {
                e.printStackTrace()
            }

            try {
                br?.forEachLine {
                    val sections = it.split(",")

                    val x = sections[0].toDouble()
                    val y = sections[1].toDouble() // TODO: Check if this needs a negative

                    pathRead.add(Translation2d(x, y))

                    val tangentX = sections[2].toDouble()
                    val tangentY = sections[3].toDouble()

                    val angle = -atan2(tangentY, tangentX)

                    angleList.add(Math.toDegrees(angle))

                }
            } catch (error: IOException) {
                // System.out.println("Ignore this error:");
                // error.printStackTrace();
            }

            var trajectoryPacket = TrajectoryPacket()

            trajectoryPacket.firstX = pathRead[0].x
            trajectoryPacket.firstY = pathRead[0].y
            trajectoryPacket.lastX = pathRead[pathRead.lastIndex].x
            trajectoryPacket.lastY = pathRead[pathRead.lastIndex].y

            pathRead.removeAt(0)
            pathRead.removeAt(pathRead.lastIndex)

            trajectoryPacket.pathRead = pathRead

            trajectoryPacket.startAngle = angleList[0]
            trajectoryPacket.endAngle = angleList[angleList.lastIndex]

            return trajectoryPacket
        }
    }
}