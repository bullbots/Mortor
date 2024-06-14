package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;

import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.locks.ReentrantLock;


public class TrajectoryManager {

    private static HashMap<String, Trajectory> trajectories;
    private final static ReentrantLock trajectoriesLock = new ReentrantLock();

    public static void generateTrajectories() {

        var genTrajectoryThread = new Thread(() -> {
            if (trajectories ==null) {

                System.out.println("INFO: Trajectories loading...");
                trajectoriesLock.lock();

                trajectories = new HashMap<>();

                var pathNames = new ArrayList<String>();

                var deployDirectory = Paths.get(Filesystem.getDeployDirectory().toString(),
                        "PathWeaver/Paths");
                var listOfFiles = deployDirectory.toFile().listFiles();

//                    for (file in listOfFiles) {
//                        pathNames.add("/" + file.name)
//                        // No filter is needed for now since onl files in deploy directory are path files.
//                    }

                for (var file : listOfFiles) {
                    // System.out.println(String.format("Adding Pathname: %s", pathName));
                    var trajPack = TrajectoryPacket.generateTrajectoryPacket(file);

                    var trajectory = TrajectoryGenerator.generateTrajectory(
                            new Pose2d(trajPack.firstX, trajPack.firstY, Rotation2d.fromDegrees(trajPack.startAngle)),
                            trajPack.pathRead,
                            new Pose2d(trajPack.lastX, trajPack.lastY, Rotation2d.fromDegrees(trajPack.endAngle)),
                            new TrajectoryConfig(6.0, 6.0).setReversed(trajPack.reversed)
//                            TrajectoryConfig(2.0, 4.0)
                    );

                    trajectories.put(file.getName(), trajectory);
                }
                trajectoriesLock.unlock();
                System.out.println("INFO: Trajectories loaded");
            }
        });

        genTrajectoryThread.setDaemon(true);
        genTrajectoryThread.start();
    }

    public static HashMap<String, Trajectory> getTrajectories() {
        HashMap<String, Trajectory> curTrajectories = null;

        if (trajectories != null) {
            System.out.println("INFO: trajectories is not null");
        }

        if (trajectories != null && trajectoriesLock.tryLock()) {
            curTrajectories = trajectories;
            trajectoriesLock.unlock();
        }

        return curTrajectories;
    }
}
