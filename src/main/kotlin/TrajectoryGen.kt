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

    val ringOffset: Pose2d = Pose2d(2.0,5.0,0.0)
    val wobbleOffset: Pose2d = Pose2d(-12.0,0.0,0.0)

    val START_WALL = Pose2d(-62.0, -42.0,Math.toRadians(180.0))
    var START_CENTER = Pose2d(-62.0, -24.0,Math.toRadians(180.0))
    var RINGS = Pose2d(-24.0, -36.0,Math.toRadians(180.0)).plus(ringOffset)
    var SHOOT = Pose2d(-2.0, -36.0,Math.toRadians(180.0)).plus(ringOffset)
    var ZONE_A = Pose2d(12.0, -60.0,Math.toRadians(0.0)).plus(wobbleOffset)
    var ZONE_B = Pose2d(36.0, -36.0,Math.toRadians(0.0)).plus(wobbleOffset)
    var ZONE_C = Pose2d(60.0, -60.0,Math.toRadians(0.0)).plus(wobbleOffset)
    var PARK = Pose2d(12.0, -36.0,Math.toRadians(180.0)).plus(ringOffset)

    var WALL_WAY = Pose2d(-24.0, -56.0,Math.toRadians(180.0))
    var WALL_WAY_START = WALL_WAY.plus(Pose2d(-15.0,4.0,0.0))
    // Configurables.  wobbleTangent should be 0.0 for ZONE_B, -45.0 otherwise
    var ZONE_VARIABLE: Pose2d = ZONE_C;
    var wobbleTangent: Double = -45.0

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        // Start with diagonal 2.5
        var trajToShoot1: Trajectory =
            TrajectoryBuilder(START_WALL, START_WALL.heading, combinedConstraints)
            //.lineTo(toVector2d(START_WALL.plus(Pose2d(10.0,-10.0,0.0))))
            .lineTo(toVector2d(WALL_WAY_START))
            .splineToLinearHeading(WALL_WAY,Math.toRadians(0.0))
            .splineToLinearHeading(SHOOT,Math.toRadians(90.0))
            .build();
        list.add(trajToShoot1)

        // Park immediately after shooting
        var trajToPark: Trajectory =
            TrajectoryBuilder(trajToShoot1.end(), trajToShoot1.end().heading, combinedConstraints)
                .splineToSplineHeading(PARK,Math.toRadians(0.0))
                .build();
        //list.add(trajToPark)

        // From shooting position to rings pickup
        var trajPickupRings: Trajectory =
            TrajectoryBuilder(trajToShoot1.end(), trajToShoot1.end().heading, combinedConstraints)
            .splineToSplineHeading(RINGS,Math.toRadians(0.0))
            .build();
        list.add(trajPickupRings)

        // Second batch of shooting after picking up rings
        var trajToShoot2: Trajectory =
            TrajectoryBuilder(trajPickupRings.end(), trajPickupRings.end().heading, combinedConstraints)
                .splineToSplineHeading(SHOOT,Math.toRadians(0.0))
                .build();
        //list.add(trajToShoot2)

        // Drive from start to Zone
        var trajStartToZone: Trajectory =
            TrajectoryBuilder(START_WALL, START_WALL.heading, combinedConstraints)
                //.lineTo(toVector2d(START_WALL.plus(Pose2d(10.0,-10.0,0.0))))
                .lineTo(toVector2d(WALL_WAY_START))
                .splineToLinearHeading(WALL_WAY,Math.toRadians(0.0))
                .splineToSplineHeading(ZONE_VARIABLE,Math.toRadians(wobbleTangent))
                .build();
        list.add(trajStartToZone)

        // Drive from shoot to Zone
        var trajFromShootToZone: Trajectory =
            TrajectoryBuilder(SHOOT, SHOOT.heading, combinedConstraints)
                //.splineTo(toVector2d(ZONE_VARIABLE),Math.toRadians(wobbleTangent))
                .lineToLinearHeading(ZONE_VARIABLE)
                .build();
        list.add(trajFromShootToZone)

        // Drive from Zone to Shoot1
        var trajZoneToShoot1: Trajectory =
            TrajectoryBuilder(trajStartToZone.end(), trajStartToZone.end().heading, combinedConstraints)
                .lineToLinearHeading(SHOOT)
                .build();
        list.add(trajZoneToShoot1)



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
