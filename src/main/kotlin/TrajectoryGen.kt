import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
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

    /*
    Offset system
     */
    val dropoffLateralOffset: Double = 4.0
    val offsetWobbleArmReach: Double = 12.0
    val offsetWobbleDropoffAlign = Pose2d(-offsetWobbleArmReach - 6.0,dropoffLateralOffset,0.0)
    val offsetWobbleDropoffDeep = Pose2d(-offsetWobbleArmReach + 2.0,dropoffLateralOffset,0.0)
    val offsetWobbleDropoffShallow = Pose2d(-offsetWobbleArmReach - 2.0,dropoffLateralOffset,0.0)
    val wobbleDropoffRotationRadians: Double = 0.0.toRadians

    val offsetWobblePickupAlign = Pose2d(-offsetWobbleArmReach - 8.0,0.0,0.0)
    val offsetWobblePickupGrab = Pose2d(-offsetWobbleArmReach + 3.0,0.0,0.0) // Drive through wobble goal
    val wobblePickupRotationRadians: Double = (165.0).toRadians

    val offsetRingPickupAlign = Pose2d(-14.0,-5.0,0.0)
    val offsetRingPickupGrab = Pose2d(-7.0,-5.0,0.0)
    val ringPickupRotationRadians: Double = (149.0).toRadians

    val spacing_powershot: Double = 7.5; // Spacing between the powershot sticks in the y axis, inches

    // old offsets
    val ringOffset: Pose2d = Pose2d(2.0,5.0,0.0)
    val wobbleOffset: Pose2d = Pose2d(-12.0,0.0,0.0)
    val wobblePickup: Pose2d = Pose2d(12.0,0.0,0.0)


    // Actual locations before offsets for robot grabber
    var RINGS_ACTUAL = Pose2d(-24.0, -36.0,Math.toRadians(0.0))  // Needs rotation for pickup
    var WOBBLE_PICKUP_ACTUAL = Pose2d(-48.0, -50.0,Math.toRadians(0.0)) // Needs rotation for pickup
    var ZONE_A_CENTER = Pose2d(12.0, -60.0,Math.toRadians(0.0))
    var ZONE_B_CENTER = Pose2d(36.0, -36.0,Math.toRadians(0.0))
    var ZONE_C_CENTER = Pose2d(60.0, -60.0,Math.toRadians(0.0))

    // Start Positions
    val START_WALL = Pose2d(-62.0, -42.0,Math.toRadians(180.0))
    var START_CENTER = Pose2d(-62.0, -18.0,Math.toRadians(180.0))

    var SHOOT_HIGHGOAL = Pose2d(-2.0,  -42.0,Math.toRadians(180.0 - 0.0))

    var POWERSHOT_CENTER = Pose2d(-2.0,  -12.0,Math.toRadians(180.0 - 0.0))
    var POWERSHOT_LEFT  = POWERSHOT_CENTER.plus(Pose2d(0.0,+1.0 * spacing_powershot, 0.0))
    var POWERSHOT_RIGHT = POWERSHOT_CENTER.plus(Pose2d(0.0,-1.0 * spacing_powershot, 0.0))


    // Offset added to rings
    var RINGS = RINGS_ACTUAL .plus(Pose2d(8.0,4.0,0.0)
            .rotateFrame(-45.0.toRadians))//.plus(ringOffset)
    var SHOOT = Pose2d(-2.0,  -42.0,Math.toRadians(180.0 - 0.0))


    // Variable waypoints
    var ZONE_CENTER_VARIABLE = ZONE_A_CENTER
    var wobbleDropoffAlign: Pose2d = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffAlign.rotateFrame(
        wobbleDropoffRotationRadians))
    var wobbleDropoffDeep: Pose2d = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffDeep.rotateFrame(
        wobbleDropoffRotationRadians))
    var wobbleDropoffShallow: Pose2d = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffShallow.rotateFrame(
        wobbleDropoffRotationRadians))
    var wobblePickupAlign: Pose2d = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupAlign.rotateFrame(
        wobblePickupRotationRadians ))
    var wobblePickupGrab: Pose2d = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupGrab.rotateFrame(
        wobblePickupRotationRadians ))
    var ringPickupAlign: Pose2d = RINGS_ACTUAL.plus(offsetRingPickupAlign.rotateFrame(ringPickupRotationRadians))
    var ringPickupGrab: Pose2d = RINGS_ACTUAL.plus(offsetRingPickupGrab.rotateFrame(ringPickupRotationRadians))


    // Older waypoints
    var CENTER_TO_SHOOT = Pose2d(-2.0, -6.0, 0.0.toRadians)
    var RIGHT_TO_SHOOT = Pose2d(-2.0, -20.0, 0.0.toRadians)
    var POWER_SHOT = Pose2d(-2.0,  -28.0,Math.toRadians(180.0 - 0.0))

    var ZONE_A = ZONE_A_CENTER.plus(wobbleOffset)
    var ZONE_B = ZONE_B_CENTER.plus(wobbleOffset)
    var ZONE_C = ZONE_C_CENTER.plus(wobbleOffset)
    var PARK = Pose2d(12.0, -42.0,Math.toRadians(180.0))
    var WOBBLE_WALL = Pose2d(-48.0, -50.0,Math.toRadians(180.0)).plus(wobblePickup)
    var PARK_RAMP = Pose2d(12.0, +10.0,Math.toRadians(0.0))


    var WALL_WAY = Pose2d(-24.0, -56.0,Math.toRadians(180.0))
    var WALL_WAY_START = WALL_WAY.plus(Pose2d(-15.0,4.0,0.0))
    var CENTER_WAY_START = Pose2d(-36.0,0.0,180.0.toRadians)
    // Configurables.  wobbleTangent should be 0.0 for ZONE_B, -45.0 otherwise
    var ZONE_VARIABLE: Pose2d = ZONE_C;
    var wobbleTangent: Double = -45.0

    // Defined trajectories
    var trajToShoot1: Trajectory? = null
    var trajToPark: Trajectory? = null
    var trajPickupRings: Trajectory? = null
    var trajToShoot2: Trajectory? = null
    var trajStartToZone: Trajectory? = null
    var trajFromShootToZone: Trajectory? = null
    var trajZoneToShoot1: Trajectory? = null
    var trajToPOWERSHOT: Trajectory? = null
    var traj_powershot_clockwise: Trajectory? = null
    // Split clockwise into 4 parts
    var traj_parkCenterToPowershotLeft: Trajectory? = null
    var traj_PowershotLeftToPowershotCenter: Trajectory? = null
    var traj_PowershotCenterPowershotRight: Trajectory? = null
    var traj_PowershotRightToWobbleDropoff: Trajectory? = null

    var trajShootToWallWobblePickup: Trajectory? = null
    var traj_startWallToStartCenter: Trajectory? = null
    var trajClaimWobbleToZone: Trajectory? = null
    var trajParkAfterWobbleDropoff: Trajectory? = null
    var trajPickupRingsFromZone: Trajectory? = null

    // Pickup rings after powershot but before first wobble pickup
    //trajPowershotRightToWobbleDropoff // Defined above
    var trajPowershotRightToRingPickupAlign: Trajectory? = null


    var trajWobbleDropoffToWobblePickupAlign: Trajectory? = null
    var trajWobbleAlignToWobblePickup: Trajectory? = null
    var trajWobblePickupToDropoffAlign: Trajectory? = null
    var trajWobbleAlignToSecondDropoff: Trajectory? = null
    var trajSecondWobbleDropoffToPark: Trajectory? = null
    var trajSecondWobbleDropoffToRingPickupAlign: Trajectory? = null
    var trajRingAlignToRingGrab: Trajectory? = null
    var trajRingGrabToShootHighGoal: Trajectory? = null
    var trajFromShootHighGoalToPark: Trajectory? = null
    // Park early if pickup fails
    var trajWobblePickupToEarlyPark: Trajectory? = null


    // 5 trajectories to support high goal and highgoal + ring pickup trajectories
    var trajCenterStartToHighGoal: Trajectory? = null
    var trajHighGoalToRingAlign: Trajectory? = null
    //trajRingAlignToRingGrab already defined (vs new unused name trajRingAlignToRingPickup)
    //trajRingGrabToShootHighGoal already defined (vs new unused name trajRingPickupToHighGoal)
    var trajHighGoalToWobbleDropoffDeep: Trajectory? = null

    val list = ArrayList<Trajectory>()



    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()
        val listPowershot = ArrayList<Trajectory>()
        val listPickupTest = ArrayList<Trajectory>()
        val listHighGoal = ArrayList<Trajectory>()
        val listWobbleFail = ArrayList<Trajectory>()
        setZone(FOUR)
        System.out.println("Angle:   " + Math.toDegrees( angleFromTo(RINGS_ACTUAL, SHOOT_HIGHGOAL)).toString())

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
        //Redundant because trajectory has been split into 4
        var traj_powershot_clockwise: Trajectory =
            trajectoryBuilder(START_CENTER, 90.0.toRadians)
                .splineToConstantHeading(CENTER_TO_SHOOT.vec(),-90.0.toRadians, combinedConstraints)
                .lineTo(RIGHT_TO_SHOOT.vec(), combinedConstraintsSlow)
                .splineToSplineHeading(ZONE_VARIABLE,Math.toRadians(wobbleTangent), combinedConstraints)
                .build();
        //list.add(traj_powershot_clockwise)
        //listPowershot.add(traj_powershot_clockwise)
        this.traj_powershot_clockwise = traj_powershot_clockwise

        // Redundant now
        //listPowershot.add(trajZoneToShoot1)


        /*
        **************************************************************
        * Rework trajectories
         */


        // Recalculate class variable waypoints based on ZONE_CENTER_VARIABLE
        wobbleDropoffAlign = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffAlign.rotateFrame(wobbleDropoffRotationRadians))
        wobbleDropoffDeep = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffDeep.rotateFrame(wobbleDropoffRotationRadians))
        wobbleDropoffShallow = ZONE_CENTER_VARIABLE.plus(offsetWobbleDropoffShallow.rotateFrame(wobbleDropoffRotationRadians))
        wobblePickupAlign = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupAlign.rotateFrame(wobblePickupRotationRadians))
        wobblePickupGrab = WOBBLE_PICKUP_ACTUAL.plus(offsetWobblePickupGrab.rotateFrame(wobblePickupRotationRadians))
        ringPickupAlign = RINGS_ACTUAL.plus(offsetRingPickupAlign.rotateFrame(ringPickupRotationRadians))
        ringPickupGrab = RINGS_ACTUAL.plus(offsetRingPickupGrab.rotateFrame(ringPickupRotationRadians))



        /* Split traj_powershot_clockwise into 4 parts
        parkCenterToPsLeft
        PsLeftToPsCenter
        PsCenterToPsRight
        PsRightToWobbleZone
         */

        var traj_parkCenterToPowershotLeft: Trajectory =
            trajectoryBuilder(START_CENTER, 0.0.toRadians)
                //.splineToConstantHeading(POWERSHOT_LEFT.vec(),45.0.toRadians)
                .lineToConstantHeading(POWERSHOT_LEFT.vec())
                .build();
        listPowershot.add(traj_parkCenterToPowershotLeft)
        this.traj_parkCenterToPowershotLeft = traj_parkCenterToPowershotLeft


        var traj_PowershotLeftToPowershotCenter: Trajectory =
            trajectoryBuilder(traj_parkCenterToPowershotLeft.end(), -90.0.toRadians)
                .lineToConstantHeading(POWERSHOT_CENTER.vec())
                .build();
        listPowershot.add(traj_PowershotLeftToPowershotCenter)
        this.traj_PowershotLeftToPowershotCenter = traj_PowershotLeftToPowershotCenter


        var traj_PowershotCenterPowershotRight: Trajectory =
            trajectoryBuilder(traj_PowershotLeftToPowershotCenter.end(), -90.0.toRadians)
                .lineToConstantHeading(POWERSHOT_RIGHT.vec())
                .build();
        listPowershot.add(traj_PowershotCenterPowershotRight)
        this.traj_PowershotCenterPowershotRight = traj_PowershotCenterPowershotRight


        /*
             Option: From powershot_right go to ring pickup
         */
        // Powershot Right to Ring Align
        var trajPowershotRightToRingPickupAlign: Trajectory =
            trajectoryBuilder(traj_PowershotCenterPowershotRight.end(), -90.0.toRadians)
                .lineToLinearHeading(ringPickupAlign)
                .build();
        listPowershot.add(trajPowershotRightToRingPickupAlign)
        listPickupTest.add(trajPowershotRightToRingPickupAlign)
        this.trajPowershotRightToRingPickupAlign = trajPowershotRightToRingPickupAlign


        // Pickup Rings
        var trajRingAlignToRingGrab: Trajectory =
            trajectoryBuilder(ringPickupAlign, 0.0.toRadians)
                .lineToConstantHeading(ringPickupGrab.vec())
                .build();
        listPowershot.add(trajRingAlignToRingGrab)
        listPickupTest.add(trajRingAlignToRingGrab)
        this.trajRingAlignToRingGrab = trajRingAlignToRingGrab


        // Take picked up rings to shoot
        var trajRingGrabToShootHighGoal: Trajectory =
            trajectoryBuilder(trajRingAlignToRingGrab.end(), 0.0.toRadians)
                .lineToConstantHeading(ringPickupAlign.vec())
                .splineToSplineHeading(SHOOT_HIGHGOAL,0.0)
                .build();
        listPowershot.add(trajRingGrabToShootHighGoal)
        listPickupTest.add(trajRingGrabToShootHighGoal)
        this.trajRingGrabToShootHighGoal = trajRingGrabToShootHighGoal


        // After shooting into highgoal, go park
        // Probably not going to use this, with pickup mid-route instead.
        var trajFromShootHighGoalToPark: Trajectory =
            trajectoryBuilder(trajRingGrabToShootHighGoal.end(), 0.0.toRadians)
                .lineToLinearHeading(PARK)
                .build();
        //listPowershot.add(trajFromShootHighGoalToPark)
        //listPickupTest.add(trajFromShootHighGoalToPark)
        this.trajFromShootHighGoalToPark = trajFromShootHighGoalToPark




        /*
            If skipping rings, go straight to wobble dropoff
         */
        var traj_PowershotRightToWobbleDropoff: Trajectory =
            when(ZONE_CENTER_VARIABLE) {
                ZONE_A_CENTER ->
                    trajectoryBuilder(traj_PowershotCenterPowershotRight.end(), -90.0.toRadians)
                        .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(0.0,6.0,0.0)),Math.toRadians(-90.0))
                        .lineToConstantHeading(wobbleDropoffDeep.vec())
                        .build();
                ZONE_B_CENTER ->
                    trajectoryBuilder(traj_PowershotCenterPowershotRight.end(), -50.0.toRadians)
                        .splineToSplineHeading(wobbleDropoffAlign,0.0)
                        .lineToConstantHeading(wobbleDropoffDeep.vec())
                        .build();
                else -> // Zone C
                    trajectoryBuilder(traj_PowershotCenterPowershotRight.end(), -40.0.toRadians)
                    // SIMPLE OPTION - turns wrong way near the wall
                        //.lineToLinearHeading(wobbleDropoffDeep)
                    // FANCY OPTION - turns away from the wall
                        .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(-15.0,5.0,1.0.toRadians)), -20.0.toRadians)
                        .splineToSplineHeading(wobbleDropoffDeep,-20.0.toRadians)
                        .build();
            }
        listPowershot.add(traj_PowershotRightToWobbleDropoff)
        this.traj_PowershotRightToWobbleDropoff = traj_PowershotRightToWobbleDropoff


        // From zone wobble dropoff position to rings pickup align
        var trajWobbleDropoffToWobblePickupAlign: Trajectory =
            when (ZONE_CENTER_VARIABLE) {
                ZONE_A_CENTER ->
                trajectoryBuilder(traj_PowershotRightToWobbleDropoff.end(), 120.0.toRadians)
                    .splineToSplineHeading(wobblePickupAlign, wobblePickupRotationRadians)
                    .build();
                ZONE_B_CENTER ->
                    trajectoryBuilder(traj_PowershotRightToWobbleDropoff.end(), 180.0.toRadians)
                        .splineToSplineHeading(wobblePickupAlign, wobblePickupRotationRadians)
                        .build();
                else ->
                    trajectoryBuilder(traj_PowershotRightToWobbleDropoff.end(), 150.0.toRadians)
                        .splineToSplineHeading(wobblePickupAlign, wobblePickupRotationRadians)
                        .build();
            }
        listPowershot.add(trajWobbleDropoffToWobblePickupAlign)
        this.trajWobbleDropoffToWobblePickupAlign = trajWobbleDropoffToWobblePickupAlign


        // Grab Wobble Goal
        var trajWobbleAlignToWobblePickup: Trajectory =
                    trajectoryBuilder(trajWobbleDropoffToWobblePickupAlign.end(), 0.0.toRadians)
                        .lineToConstantHeading(wobblePickupGrab.vec())
                        .build();
        listPowershot.add(trajWobbleAlignToWobblePickup)
        this.trajWobbleAlignToWobblePickup = trajWobbleAlignToWobblePickup


        // Align to dropoff second wobble goal
        var trajWobblePickupToDropoffAlign: Trajectory =
            when(ZONE_CENTER_VARIABLE) {
                ZONE_A_CENTER ->
                    trajectoryBuilder(trajWobbleAlignToWobblePickup.end(), wobblePickupRotationRadians + 180.0.toRadians)
                        .splineToSplineHeading(wobbleDropoffAlign,Math.toRadians(-30.0))
                        //.lineToConstantHeading(wobbleDropoffAlign.vec())
                        .build();
                ZONE_B_CENTER ->
                    trajectoryBuilder(trajWobbleAlignToWobblePickup.end(), wobblePickupRotationRadians + 180.0.toRadians)
                        .lineToConstantHeading(wobblePickupAlign.vec())
                        .splineToSplineHeading(wobbleDropoffAlign,25.0.toRadians)
                        //.lineToConstantHeading(wobbleDropoffDeep.vec())
                        .build();
                else -> // Zone C
                    trajectoryBuilder(trajWobbleAlignToWobblePickup.end(), wobblePickupRotationRadians + (180.0).toRadians)
                        //.lineToConstantHeading(wobblePickupAlign.vec())
                        .splineToSplineHeading(wobbleDropoffAlign,-20.0.toRadians)
                        .build();
            }
        listPowershot.add(trajWobblePickupToDropoffAlign)
        this.trajWobblePickupToDropoffAlign = trajWobblePickupToDropoffAlign


        // If 2nd wobble pickup fails, park early
        var trajWobblePickupToEarlyPark: Trajectory =
            trajectoryBuilder(trajWobbleAlignToWobblePickup.end(), wobblePickupRotationRadians + 180.0.toRadians)
                .lineToConstantHeading(wobblePickupAlign.vec())
                .splineToSplineHeading(PARK_RAMP,90.0.toRadians)
                .build();
        listPowershot.add(trajWobblePickupToEarlyPark)
        listWobbleFail.add(trajWobblePickupToEarlyPark)
        this.trajWobblePickupToEarlyPark = trajWobblePickupToEarlyPark



        // Dropoff Second WobbleGoal
        var trajWobbleAlignToSecondDropoff: Trajectory =
            trajectoryBuilder(trajWobblePickupToDropoffAlign.end(), 0.0.toRadians)
                .lineToConstantHeading(wobbleDropoffShallow.vec())
                .build();
        listPowershot.add(trajWobbleAlignToSecondDropoff)
        this.trajWobbleAlignToSecondDropoff = trajWobbleAlignToSecondDropoff


        // Second WobbleGoal to Park (best option)
        var trajSecondWobbleDropoffToPark: Trajectory =
            when(ZONE_CENTER_VARIABLE) {
                ZONE_A_CENTER ->
                trajectoryBuilder(trajWobbleAlignToSecondDropoff.end(), 180.0.toRadians)
                    .lineToConstantHeading( wobbleDropoffAlign.vec() )
                    .splineToConstantHeading(PARK.vec().plus(Vector2d(0.0,12.0)),0.0.toRadians)
                    .build();
                else ->
                trajectoryBuilder(trajWobbleAlignToSecondDropoff.end(), 135.0.toRadians)
                    .lineToConstantHeading(PARK.vec().plus(Vector2d(0.0,12.0)))
                    .build();
            }
        listPowershot.add(trajSecondWobbleDropoffToPark)
        this.trajSecondWobbleDropoffToPark = trajSecondWobbleDropoffToPark


        // Second WobbleGoal to Ring Pickup Align
        // Probably not going to use this, with pickup mid-route instead.
        var trajSecondWobbleDropoffToRingPickupAlign: Trajectory =
            when(ZONE_CENTER_VARIABLE) {
                ZONE_A_CENTER ->
                    trajectoryBuilder(trajWobbleAlignToSecondDropoff.end(), 180.0.toRadians)
                        .lineToConstantHeading( wobbleDropoffAlign.vec().plus(Vector2d(0.0,7.0)) )
                        .splineToSplineHeading( ringPickupAlign, 90.0.toRadians + 0.0*ringPickupRotationRadians)
                        //.lineToLinearHeading( ringPickupAlign)
                        .build();
                else ->
                    trajectoryBuilder(trajWobbleAlignToSecondDropoff.end(), 180.0.toRadians)
                        .lineToLinearHeading(ringPickupAlign)
                        .build();
            }
        //listPowershot.add(trajSecondWobbleDropoffToRingPickupAlign)
        //listPickupTest.add(trajSecondWobbleDropoffToRingPickupAlign)
        this.trajSecondWobbleDropoffToRingPickupAlign = trajSecondWobbleDropoffToRingPickupAlign




        // From zone wobble dropoff position to rings pickup
        // Probably not going to use this, with pickup mid-route instead.
        var trajPickupRingsFromZone: Trajectory =
            trajectoryBuilder(traj_powershot_clockwise.end(), (wobbleTangent + 180.0).toRadians)
                .splineToSplineHeading(SHOOT,180.0.toRadians)
                .splineTo(toVector2d(RINGS),Math.toRadians(180.0 - 45.0))
                .build();
        //list.add(trajPickupRings)
        //listPowershot.add(trajPickupRingsFromZone)
        this.trajPickupRingsFromZone = trajPickupRingsFromZone

        ///listPowershot.add(trajToShoot2)

        // Claim wall wobble goal
        var trajShootToWallWobblePickup: Trajectory =
            trajectoryBuilder(trajZoneToShoot1.end(),trajZoneToShoot1.end().heading)
                .splineToLinearHeading(WALL_WAY,-180.0.toRadians)
                //.splineTo(WALL_WAY_START.vec(),-180.0.toRadians)
                .splineToLinearHeading(WOBBLE_WALL,(-180.0 - 0.0*45.0).toRadians)
                .build();
        //list.add(traj_powershot_clockwise)
        //listPowershot.add(trajShootToWallWobblePickup)
        this.trajShootToWallWobblePickup = trajShootToWallWobblePickup

        // From Claim Wobble to Zone
        var trajClaimWobbleToZone: Trajectory =
            trajectoryBuilder(trajShootToWallWobblePickup.end(),-20.0.toRadians)
                //.splineToLinearHeading(SHOOT, 0.0)
                .splineToSplineHeading(ZONE_VARIABLE, wobbleTangent.toRadians)
                .build();
        //list.add(trajClaimWobbleToZone)
        //listPowershot.add(trajClaimWobbleToZone)
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
        //listPowershot.add(trajParkAfterWobbleDropoff)
        this.trajParkAfterWobbleDropoff = trajParkAfterWobbleDropoff


        /*
           5 new trajectories for doing high shot and optionally rings pickup
            trajCenterStartToHighGoal
            trajHighGoalToRingAlign
            trajRingAlignToRingGrab // Already defined
            trajRingGrabToShootHighGoal // Already defined
            trajHighGoalToWobbleDropoffDeep
         */

        // Start Center to HighGoal shoot position
        // Calculate x position for smooth turn around rings
        val xPositionWeighted = (2.0 * RINGS_ACTUAL.x + 1.0 * SHOOT_HIGHGOAL.x) / 3.0
        val ringLeftToHighGoal = Pose2d(xPositionWeighted, START_CENTER.y,0.0.toRadians);
        var trajCenterStartToHighGoal: Trajectory =
            trajectoryBuilder(START_CENTER, 0.0.toRadians)
                .splineTo(ringLeftToHighGoal.vec(), 0.0.toRadians)
                .splineToLinearHeading(SHOOT_HIGHGOAL, -80.0.toRadians) // Approach direction
                //.splineTo(ringLeftToHighGoal)
                //.(SHOOT_HIGHGOAL)
                //.lineToLinearHeading(SHOOT_HIGHGOAL)
                .build();
        listHighGoal.add(trajCenterStartToHighGoal)
        this.trajCenterStartToHighGoal = trajCenterStartToHighGoal


        // High Goal to Ring Align
        var trajHighGoalToRingAlign: Trajectory =
            trajectoryBuilder(SHOOT_HIGHGOAL, 180.0.toRadians)
                .lineToLinearHeading(ringPickupAlign)
                .build();
        listHighGoal.add(trajHighGoalToRingAlign)
        this.trajHighGoalToRingAlign = trajHighGoalToRingAlign


        // RingAlign to  Ring Pickup     --- OLD TRAJECTORY
        // trajRingAlignToRingGrab
        listHighGoal.add(trajRingAlignToRingGrab)


        // Ring pickup to high goal     --- OLD TRAJECTORY
        // trajRingGrabToShootHighGoal
        listHighGoal.add(trajRingGrabToShootHighGoal)


        // High Goal to Wobble Drop (NOT created elsewhere)
        var trajHighGoalToWobbleDropoffDeep: Trajectory =
        when(ZONE_CENTER_VARIABLE) {
            ZONE_A_CENTER ->
                trajectoryBuilder(trajRingGrabToShootHighGoal.end(), -60.0.toRadians)
                // SIMPLE OPTION - turns wrong way near the wall
                    //.splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(0.0,6.0,0.0)),Math.toRadians(-90.0))
                    //.lineToConstantHeading(wobbleDropoffDeep.vec())
                // FANCY OPTION - turns away from the wall
                    .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(0.0,6.0,1.0.toRadians)),Math.toRadians(-90.0))
                    .lineToSplineHeading(wobbleDropoffDeep)
                    .build();
            ZONE_B_CENTER ->
                trajectoryBuilder(trajRingGrabToShootHighGoal.end(), 30.0.toRadians)
                    //.lineToLinearHeading(wobbleDropoffDeep)
                    .splineToSplineHeading(wobbleDropoffAlign,0.0)
                    .lineToConstantHeading(wobbleDropoffDeep.vec())
                    .build();
            else -> // Zone C
                trajectoryBuilder(trajRingGrabToShootHighGoal.end(), -20.0.toRadians)
                // SIMPLE OPTION - turns wrong way near the wall
                    //.lineToLinearHeading(wobbleDropoffDeep)
                // FANCY OPTION - turns away from the wall
                    .splineToSplineHeading(wobbleDropoffDeep.plus(Pose2d(-20.0,3.0,1.0.toRadians)), -20.0.toRadians)
                    .splineToSplineHeading(wobbleDropoffDeep,0.0.toRadians)
                    .build();
        }
        listHighGoal.add(trajHighGoalToWobbleDropoffDeep)
        this.trajHighGoalToWobbleDropoffDeep = trajHighGoalToWobbleDropoffDeep
        listPickupTest.add(trajHighGoalToWobbleDropoffDeep)
        listPowershot.add(trajHighGoalToWobbleDropoffDeep)


        // Adding the remaining powershot trajectories after first wobble dropoff
        listHighGoal.addAll(listPowershot.subList(8,13))


        //return list
        return listPowershot
        //return listPickupTest
        //return listHighGoal
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
        ZONE_CENTER_VARIABLE = when (ringAmount) {
            ZERO -> ZONE_A_CENTER
            ONE -> ZONE_B_CENTER
            FOUR -> ZONE_C_CENTER
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

    fun angleFromTo(fromPose: Vector2d, toPose: Vector2d): Double {
       return Math.atan2(toPose.y - fromPose.y, toPose.x - fromPose.x);
    }

    fun angleFromTo(fromPose: Pose2d, toPose: Pose2d): Double {
        return angleFromTo(fromPose.vec(),toPose.vec());
    }
}


val Double.toRadians get() = (Math.toRadians(this))

/*
    +x is the 'positive' direction, and rotation is counter-clockwise around (0,0)
    https://en.wikipedia.org/wiki/Rotation_matrix
 */
fun Pose2d.rotateFrame(rotationRadians: Double): Pose2d
{
    return Pose2d(this.x * kotlin.math.cos(rotationRadians) - this.y * kotlin.math.sin(rotationRadians),
        this.x * kotlin.math.sin(rotationRadians) + this.y * kotlin.math.cos(rotationRadians),
        this.heading + rotationRadians)
}