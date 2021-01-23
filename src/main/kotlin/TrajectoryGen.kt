import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import javafx.scene.paint.Color
import javafx.scene.paint.Color.color
import java.util.*
import kotlin.collections.ArrayList
import TrajectoryGen.RingDetectionAmount.*

object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(60.0, 60.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)
    private val driveConstraintsSlow = DriveConstraints(10.0, 60.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 16.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)
    private val combinedConstraintsSlow = MecanumConstraints(driveConstraintsSlow, trackWidth)

    val ringOffset: Pose2d = Pose2d(2.0,5.0,0.0)
    val wobbleOffset: Pose2d = Pose2d(-12.0,0.0,0.0)
    val wobblePickup: Pose2d = Pose2d(12.0,0.0,0.0)

    val START_WALL = Pose2d(-62.0, -42.0,Math.toRadians(180.0))
    var START_CENTER = Pose2d(-62.0, -18.0,Math.toRadians(180.0))
    var RINGS = Pose2d(-24.0, -36.0,Math.toRadians(180.0)).plus(ringOffset)
    var SHOOT = Pose2d(-2.0,  -42.0,Math.toRadians(180.0 - 0.0))
    var CENTER_TO_SHOOT = Pose2d(-2.0, -6.0, 0.0.toRadians)
    var RIGHT_TO_SHOOT = Pose2d(-2.0, -20.0, 0.0.toRadians)
    var POWER_SHOT = Pose2d(-2.0,  -28.0,Math.toRadians(180.0 - 0.0))
    var ZONE_A = Pose2d(12.0, -60.0,Math.toRadians(0.0)).plus(wobbleOffset)
    var ZONE_B = Pose2d(36.0, -36.0,Math.toRadians(0.0)).plus(wobbleOffset)
    var ZONE_C = Pose2d(60.0, -60.0,Math.toRadians(0.0)).plus(wobbleOffset)
    var PARK = Pose2d(12.0, -42.0,Math.toRadians(180.0))
    var WOBBLE_WALL = Pose2d(-48.0, -50.0,Math.toRadians(180.0)).plus(wobblePickup)


    var WALL_WAY = Pose2d(-24.0, -56.0,Math.toRadians(180.0))
    var WALL_WAY_START = WALL_WAY.plus(Pose2d(-15.0,4.0,0.0))
    var CENTER_WAY_START = Pose2d(-36.0,0.0,180.0.toRadians)
    // Configurables.  wobbleTangent should be 0.0 for ZONE_B, -45.0 otherwise
    var ZONE_VARIABLE: Pose2d = ZONE_C;
    var wobbleTangent: Double = -45.0

    var trajToShoot1: Trajectory? = null
    var trajToPark: Trajectory? = null
    var trajPickupRings: Trajectory? = null
    var trajToShoot2: Trajectory? = null
    var trajStartToZone: Trajectory? = null
    var trajFromShootToZone: Trajectory? = null
    var trajZoneToShoot1: Trajectory? = null
    var trajToPOWERSHOT: Trajectory? = null
    var traj_powershot_clockwise: Trajectory? = null
    var trajShootToWallWobblePickup: Trajectory? = null
    var traj_startWallToStartCenter: Trajectory? = null
    var trajClaimWobbleToZone: Trajectory? = null
    var trajParkAfterWobbleDropoff: Trajectory? = null
    var trajPickupRingsFromZone: Trajectory? = null

    val list = ArrayList<Trajectory>()



    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()
        val listPowershot = ArrayList<Trajectory>()
        setZone(ZERO)
        // Note that powershot path ZERO doesn't park well

        // Start with diagonal 2.5
        var trajToShoot1: Trajectory =
            trajectoryBuilder(START_WALL, START_WALL.heading)
            //.lineTo(toVector2d(START_WALL.plus(Pose2d(10.0,-10.0,0.0))))
            .lineTo(toVector2d(WALL_WAY_START))
            .splineToLinearHeading(WALL_WAY,Math.toRadians(0.0))
            .splineToLinearHeading(SHOOT,Math.toRadians(90.0))
            //.lineTo(toVector2d(SHOOT))
            .build();
        list.add(trajToShoot1)
        this.trajToShoot1 = trajToShoot1

        // Powershot!!!
        var trajToPOWERSHOT: Trajectory =
            trajectoryBuilder(START_WALL, START_WALL.heading)
                //.lineTo(toVector2d(START_WALL.plus(Pose2d(10.0,-10.0,0.0))))
                .lineTo(toVector2d(WALL_WAY_START))
                .splineToLinearHeading(WALL_WAY,Math.toRadians(0.0))
                .splineToLinearHeading(POWER_SHOT.plus(Pose2d(0.0,24.0,0.0)),Math.toRadians(90.0))
                //.lineTo(toVector2d(SHOOT))
                .build();
        list.add(trajToPOWERSHOT)
        this.trajToPOWERSHOT = trajToPOWERSHOT

        // Park immediately after shooting
        var trajToPark: Trajectory =
            trajectoryBuilder(trajToShoot1.end(), trajToShoot1.end().heading* 0.0)
                .splineToSplineHeading(PARK,Math.toRadians(0.0))
                .build();
        list.add(trajToPark)
        this.trajToPark = trajToPark

        // From shooting position to rings pickup
        var trajPickupRings: Trajectory =
            trajectoryBuilder(trajToShoot1.end(), trajToShoot1.end().heading)
            //.splineToSplineHeading(RINGS,Math.toRadians(180.0))
            //.splineToLinearHeading(RINGS,Math.toRadians(180.0))
            .splineTo(toVector2d(RINGS),Math.toRadians(180.0 - 45.0))
            .build();
        list.add(trajPickupRings)
        this.trajPickupRings = trajPickupRings

        // Second batch of shooting after picking up rings
        var trajToShoot2: Trajectory =
            trajectoryBuilder(trajPickupRings.end(), trajPickupRings.end().heading + Math.toRadians(180.0))
                //.splineToSplineHeading(SHOOT,Math.toRadians(0.0))
                .splineToLinearHeading(SHOOT,Math.toRadians(0.0))
                .build();
        list.add(trajToShoot2)
        this.trajToShoot2 = trajToShoot2

        // Drive from start to Zone
        var trajStartToZone: Trajectory =
            trajectoryBuilder(START_WALL, START_WALL.heading)
                //.lineTo(toVector2d(START_WALL.plus(Pose2d(10.0,-10.0,0.0))))
                .lineTo(toVector2d(WALL_WAY_START))
                .splineToLinearHeading(WALL_WAY,Math.toRadians(0.0))
                .splineToSplineHeading(ZONE_VARIABLE,Math.toRadians(wobbleTangent))
                .build();
        //list.add(trajStartToZone)
        //this.trajStartToZone = trajStartToZone

        // Drive from shoot to Zone
        var trajFromShootToZone: Trajectory =
            trajectoryBuilder(SHOOT, SHOOT.heading)
                //.splineTo(toVector2d(ZONE_VARIABLE),Math.toRadians(wobbleTangent))
                .lineToLinearHeading(ZONE_VARIABLE)
                .build();
        //list.add(trajFromShootToZone)
        this.trajFromShootToZone = trajFromShootToZone

        // Drive from Zone to Shoot1
        var trajZoneToShoot1: Trajectory =
            trajectoryBuilder(trajStartToZone.end(), trajStartToZone.end().heading)
                .lineToLinearHeading(SHOOT)
                .build();
        list.add(trajZoneToShoot1)
        this.trajZoneToShoot1 = trajZoneToShoot1

        // Drive Home, for convenience.
        val wallY = -56.0
        val resetHeading = Math.toRadians(180.0)
        val testEnd: Pose2d = Pose2d(30.0, 10.0,15.0.toRadians)
        val startWall = Pose2d(-62.0, -42.0, 180.0.toRadians)
        val wallGoal = Pose2d(testEnd.x, wallY, resetHeading)
        val wallStart = Pose2d(startWall.x + 20.0, wallY, resetHeading)

        val traj_home: Trajectory = trajectoryBuilder(testEnd, Math.toRadians(180.0))
            .splineToSplineHeading(wallGoal, Math.toRadians(180.0))
            .splineToSplineHeading(wallStart, Math.toRadians(180.0))
            .splineToLinearHeading(startWall, Math.toRadians(180.0))
            .build()
        //list.add(traj_home)



        // Clockwise powershot tour
        // Demo relative start position placement
        var traj_startWallToStartCenter: Trajectory =
            trajectoryBuilder(START_WALL, (90.0 - 20.0).toRadians)
                .splineToConstantHeading(START_CENTER.vec(),(20.0 + 90.0).toRadians)
                .build();
        //list.add(traj_startWallToStartCenter)
        listPowershot.add(traj_startWallToStartCenter)
        this.traj_startWallToStartCenter = traj_startWallToStartCenter


        //setZone(ONE)
        var traj_powershot_clockwise: Trajectory =
            trajectoryBuilder(START_CENTER, 90.0.toRadians)
                .splineToConstantHeading(CENTER_TO_SHOOT.vec(),-90.0.toRadians, combinedConstraints)
                .lineTo(RIGHT_TO_SHOOT.vec(), combinedConstraintsSlow)
                .splineToSplineHeading(ZONE_VARIABLE,Math.toRadians(wobbleTangent), combinedConstraints)
                .build();
        //list.add(traj_powershot_clockwise)
        listPowershot.add(traj_powershot_clockwise)
        this.traj_powershot_clockwise = traj_powershot_clockwise

        // Redundant now
        //listPowershot.add(trajZoneToShoot1)

        // From zone wobble dropoff position to rings pickup
        var trajPickupRingsFromZone: Trajectory =
            trajectoryBuilder(traj_powershot_clockwise.end(), (wobbleTangent + 180.0).toRadians)
                .splineToSplineHeading(SHOOT,180.0.toRadians)
                .splineTo(toVector2d(RINGS),Math.toRadians(180.0 - 45.0))
                .build();
        //list.add(trajPickupRings)
        listPowershot.add(trajPickupRingsFromZone)
        this.trajPickupRingsFromZone = trajPickupRingsFromZone

        listPowershot.add(trajToShoot2)

        // Claim wall wobble goal
        var trajShootToWallWobblePickup: Trajectory =
            trajectoryBuilder(trajZoneToShoot1.end(),trajZoneToShoot1.end().heading)
                .splineToLinearHeading(WALL_WAY,-180.0.toRadians)
                //.splineTo(WALL_WAY_START.vec(),-180.0.toRadians)
                .splineToLinearHeading(WOBBLE_WALL,(-180.0 - 0.0*45.0).toRadians)
                .build();
        //list.add(traj_powershot_clockwise)
        listPowershot.add(trajShootToWallWobblePickup)
        this.trajShootToWallWobblePickup = trajShootToWallWobblePickup

        // From Claim Wobble to Zone
        var trajClaimWobbleToZone: Trajectory =
            trajectoryBuilder(trajShootToWallWobblePickup.end(),-20.0.toRadians)
                //.splineToLinearHeading(SHOOT, 0.0)
                .splineToSplineHeading(ZONE_VARIABLE, wobbleTangent.toRadians)
                .build();
        //list.add(trajClaimWobbleToZone)
        listPowershot.add(trajClaimWobbleToZone)
        this.trajClaimWobbleToZone = trajClaimWobbleToZone

        // Park after wobble zone dropoff
        // Note variable trajectory construction using when to prevent knocking wobble goal from ZONE A.
        var trajParkAfterWobbleDropoff: Trajectory =
            when (ZONE_VARIABLE) {
                ZONE_A ->
                    trajectoryBuilder(trajClaimWobbleToZone.end(),trajClaimWobbleToZone.end().heading + 180.0.toRadians)
                        .splineToConstantHeading(SHOOT.vec(),0.0.toRadians)
                        .lineTo(PARK.vec())
                        .build()
                else ->
                    trajectoryBuilder(trajClaimWobbleToZone.end(),trajClaimWobbleToZone.end().heading + 180.0.toRadians)
                        .lineTo(PARK.vec())
                        .build()
            };


        //list.add(trajParkAfterWobbleDropoff)
        listPowershot.add(trajParkAfterWobbleDropoff)
        this.trajParkAfterWobbleDropoff = trajParkAfterWobbleDropoff

        //return list
        return listPowershot
    }

    fun drawOffbounds() {
        //GraphicsUtil.setColor(Color.color(0.0,0.0,200.0))
        GraphicsUtil.fillRect(SHOOT.vec(), 18.0, 18.0) // robot against the wall
    }

    fun toVector2d(pose: Pose2d): Vector2d {
        return Vector2d(pose.x,pose.y)
    }


    fun  trajectoryBuilder(pose: Pose2d, heading: Double): TrajectoryBuilder{
        return TrajectoryBuilder(pose, heading, combinedConstraints)
    }

    fun  trajectoryBuilder(pose: Pose2d, reversed: Boolean): TrajectoryBuilder{
        return TrajectoryBuilder(pose, reversed, combinedConstraints)
    }

    fun setZone(ringAmount: RingDetectionAmount) {
        ZONE_VARIABLE = when (ringAmount) {
            ZERO -> ZONE_A
            ONE -> ZONE_B
            FOUR -> ZONE_C
        }
        wobbleTangent = when (ringAmount) {
            ZERO -> -40.0
            ONE -> 0.0
            FOUR -> -15.0
        }
        //buildTrajectories()
    }

    enum class RingDetectionAmount {
        ZERO, ONE, FOUR
    }

}


val Double.toRadians get() = (Math.toRadians(this))

