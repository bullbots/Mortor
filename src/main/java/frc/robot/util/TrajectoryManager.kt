package frc.robot.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.Filesystem
import java.nio.file.Paths
import java.util.concurrent.locks.ReentrantLock


class TrajectoryManager {

    companion object {
        private var trajectories: HashMap<String, Trajectory>? = null
        private val trajectoriesLock = ReentrantLock()

        fun generateTrajectories() {

            val genTrajectoryThread = Thread {
                if (trajectories == null) {
                    println("INFO: Trajectories loading...")
                    trajectoriesLock.lock()

                    trajectories = HashMap<String, Trajectory>()

                    val pathNames = ArrayList<String>()

                    val deployDirectory = Paths.get(Filesystem.getDeployDirectory().toString(), "PathWeaver/Paths")
                    val listOfFiles = deployDirectory.toFile().listFiles()

//                    for (file in listOfFiles) {
//                        pathNames.add("/" + file.name)
//                        // No filter is needed for now since onl files in deploy directory are path files.
//                    }

                    for (file in listOfFiles) {
                        // System.out.println(String.format("Adding Pathname: %s", pathName));
                        val trajPack = TrajectoryPacket.generateTrajectoryPacket(file)

                        val trajectory = TrajectoryGenerator.generateTrajectory(
                            Pose2d(trajPack.firstX, trajPack.firstY, Rotation2d.fromDegrees(trajPack.startAngle)),
                            trajPack.pathRead,
                            Pose2d(trajPack.lastX, trajPack.lastY, Rotation2d.fromDegrees(trajPack.endAngle)),
//                            TrajectoryConfig(6.5, 6.5).setReversed(trajPack.reversed)
                            TrajectoryConfig(2.0, 2.0).setReversed(trajPack.reversed)
//                            TrajectoryConfig(2.0, 4.0)
                        )

                        trajectories!![file.name] = trajectory
                    }
                    trajectoriesLock.unlock()
                    println("INFO: Trajectories loaded")


                }
            }

            genTrajectoryThread.isDaemon = true
            genTrajectoryThread.start()
        }

        fun getTrajectories(): HashMap<String, Trajectory>? {
            var curTrajectories: HashMap<String, Trajectory>? = null

            if (trajectories != null) {
                println("INFO: trajectories is not null")
            }

            if (trajectories != null && trajectoriesLock.tryLock()) {
                curTrajectories = trajectories
                trajectoriesLock.unlock()
            }

            return curTrajectories
        }
    }
}