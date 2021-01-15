import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints

object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(60.0, 60.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 16.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    val START_WALL = Pose2d(-62.0, -42.0,Math.toRadians(180.0))
    var START_CENTER = Pose2d(-62.0, -24.0,Math.toRadians(180.0))
    var RINGS = Pose2d(-24.0, -36.0,Math.toRadians(180.0))
    var SHOOT = Pose2d(-2.0, -36.0,Math.toRadians(180.0))
    var ZONE_A = Pose2d(12.0, -60.0,Math.toRadians(180.0)) // could be -90 so overshooting hits the wall

    var ZONE_B = Pose2d(-36.0, 36.0,Math.toRadians(0.0))
    var ZONE_C = Pose2d(-60.0, 60.0,Math.toRadians(0.0))
    var PARK = Pose2d(12.0, -36.0,Math.toRadians(0.0))

    var TO_ZONE = Pose2d(-24.0, -60.0,Math.toRadians(180.0))
    var ZONE_VARIABLE: Pose2d? = null

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        //Constant heading spline
        val builder3 = TrajectoryBuilder(START_WALL, START_WALL.heading, combinedConstraints)
        builder3
            .back(1.0)
            .splineToLinearHeading(TO_ZONE,Math.toRadians(0.0))
            .splineToLinearHeading(SHOOT,Math.toRadians(90.0))
            .build();
        list.add(builder3.build())

        // Cool looking trajectory
        val builder4 = TrajectoryBuilder(START_WALL, START_WALL.heading, combinedConstraints)
        builder4
            .back(1.0)
            .splineTo(toVector2d(TO_ZONE),Math.toRadians(0.0))
            .splineToSplineHeading(SHOOT,Math.toRadians(90.0))
            .build();
        list.add(builder4.build())

        // Start with diagonal
        val builder5 = TrajectoryBuilder(START_WALL, START_WALL.heading, combinedConstraints)
        builder5
            .lineTo(toVector2d(START_WALL.plus(Pose2d(10.0,-10.0,0.0))))
            .splineToLinearHeading(TO_ZONE,Math.toRadians(0.0))
            .splineToLinearHeading(SHOOT,Math.toRadians(90.0))
            .build();
        list.add(builder5.build())




        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }

    fun toVector2d(pose: Pose2d): Vector2d {
        return Vector2d(pose.x,pose.y)
    }
}


val Double.toRadians get() = (Math.toRadians(this))
