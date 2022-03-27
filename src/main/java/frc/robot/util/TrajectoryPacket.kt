package frc.robot.util

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Filesystem
import java.io.*
import kotlin.math.atan2


class TrajectoryPacket {

    var firstX = 0.0
    var firstY = 0.0

    lateinit var pathRead: ArrayList<Translation2d>

    var lastX = 0.0
    var lastY = 0.0

    var startAngle = 0.0
    var endAngle = 0.0

    var reversed = false

    companion object {

        fun generateTrajectoryPacket(file: File): TrajectoryPacket {

            var pathRead = ArrayList<Translation2d>()
            var angleList = ArrayList<Double>()

            var br: BufferedReader? = null

            try {
                println("INFO: reading file: $file")
                br = BufferedReader(FileReader(file))
            } catch (e: FileNotFoundException) {
                e.printStackTrace()
            }

            var reversed = false

            try {
                br?.forEachLine {
                    val sections = it.split(",")

                    if (sections[0] == "X") {
                        return@forEachLine
                    }

                    val x = sections[0].toDouble()
                    val y = sections[1].toDouble() + 27 // TODO: Check if this needs a negative

                    pathRead.add(Translation2d(x, y))

                    val tangentX = sections[2].toDouble()
                    val tangentY = sections[3].toDouble()

                    val angle = atan2(tangentY, tangentX)

                    angleList.add(Math.toDegrees(angle))

                    reversed = sections[5].toBoolean()
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

            trajectoryPacket.reversed = reversed

            return trajectoryPacket
        }
    }
}