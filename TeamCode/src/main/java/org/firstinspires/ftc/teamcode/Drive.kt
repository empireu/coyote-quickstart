package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.hardware.*
import empireu.coyote.*
import org.firstinspires.ftc.teamcode.dashboard.DriveConfig.*
import org.firstinspires.ftc.teamcode.dashboard.LocalizerConfig
import kotlin.math.absoluteValue

fun feedForward(velocity: Double, acceleration: Double) =
    velocity * Kv + acceleration * Ka + Ks.signed(velocity)

fun feedForwardDual(velocity: Dual): Double =
    feedForward(velocity.value, velocity.tail().value)

fun vCompFeedForward(velocity: Double, acceleration: Double, voltage: Double) =
    feedForward(velocity, acceleration) / voltage

fun vCompFeedForwardDual(velocity: Dual, voltage: Double): Double =
    vCompFeedForward(velocity.value, velocity.tail().value, voltage)

fun holoCommand(targetPosWorld: Pose2dDual, actualPosWorld: Pose2d): Twist2dDual {
    val posError = targetPosWorld.value / actualPosWorld

    // (tf = transform notation) world -> robot
    val tfWorldRobot = Pose2dDual.const(actualPosWorld.inverse, 3)
    val targetVelRobot = tfWorldRobot * targetPosWorld.velocity

    return Twist2dDual(
            targetVelRobot.trVelocity.x - KPosX * posError.translation.x,
            targetVelRobot.trVelocity.y - KPosY * posError.translation.y,
            targetVelRobot.rotVelocity + KPosR * posError.rotation.log()
    )
}

class HoloTrajectoryController(val trajectory: Trajectory) {
    private val watch = Stopwatch()

    var isWithinTolerance = false
        private set

    var targetState = trajectory.evaluate(0.0)
        private set

    fun update(actualPosWorld: Pose2d): Twist2dDual {
        val t = watch.total
        targetState = trajectory.evaluate(t)

        val posError = actualPosWorld / targetState.pose

        if(
            t >= trajectory.timeEnd &&
            posError.translation.length < AdmissibleDistance &&
            posError.rotation.log().absoluteValue < Math.toRadians(AdmissibleAngleDeg))
        {
            isWithinTolerance = true
        }

        return holoCommand(
            targetPosWorld = Pose2dDual(

                // Oof, we have some mistakes in the trajectory velocity/acceleration calculations.
                // The solution would be to go back and fix those.
                // The wrong values are easily visualized using the "Kinematic Analysis" tool.

                Vector2dDual(
                    Dual.of(targetState.pose.translation.x, -targetState.velocity.x, -targetState.acceleration.x),
                    Dual.of(targetState.pose.translation.y, -targetState.velocity.y, -targetState.acceleration.y)
                ),

                Rotation2dDual.exp(
                    Dual.of(targetState.pose.rotation.log(), -targetState.angularVelocity, -targetState.angularAcceleration)
                )
            ),
            actualPosWorld
        )
    }
}

class MecanumDrive(hardwareMap: HardwareMap) : IDriveController {
    val motorFL: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "MotorFL")
    val motorFR: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "MotorFR")
    val motorBL: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "MotorBL")
    val motorBR: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "MotorBR")
    val motors = listOf(motorFL, motorFR, motorBL, motorBR)

    val voltageSensor: VoltageSensor = hardwareMap.voltageSensor.first()

    private val localizer: ILocalizer

    var position = Pose2d(0.0, 0.0, 0.0)
        private set

    private val poseHistory = ArrayList<Pose2d>()
    private var controller: HoloTrajectoryController? = null

    init {
        motors.forEach { motor ->
            motor.motorType = motor.motorType.clone().also {
                it.achieveableMaxRPMFraction = 1.0
            }

            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        motorFL.direction = DcMotorSimple.Direction.REVERSE
        motorBL.direction = DcMotorSimple.Direction.REVERSE

        localizer = Holo3WheelLocalizer(hardwareMap)
    }

    fun applyDrivePower(power: Twist2d) {
        if(power.approxEqs(Twist2d.zero, 10e-6)){
            resetPower()

            return
        }

        applyVelocity(
            Twist2dDual.const(
                Twist2d(
                    power.trVelocity * DriveTVel,
                    power.rotVelocity * Math.toRadians(DriveRVelDeg)
                ), 2
            )
        )
    }

    fun applyVelocity(velRobot: Twist2dDual) {
        val v = MecanumKinematics.inverse(velRobot, A, B)
        val voltage = voltageSensor.voltage

        motorFL.power = vCompFeedForwardDual(v.frontLeft, voltage)
        motorFR.power = vCompFeedForwardDual(v.frontRight, voltage)
        motorBL.power = vCompFeedForwardDual(v.backLeft, voltage)
        motorBR.power = vCompFeedForwardDual(v.backRight, voltage)
    }

    fun resetPower() {
        motors.forEach { it.power = 0.0 }
    }

    fun update() {
        position += localizer.readIncr()

        poseHistory.add(position)

        if(poseHistory.size > 500){
            poseHistory.removeAt(0)
        }

        val packet = TelemetryPacket()

        packet.addLine("$position")

        val overlay = packet.fieldOverlay()

        val controller = this.controller

        if(controller != null) {
            applyVelocity(
                controller.update(
                    actualPosWorld = position
                )
            )

            DashboardUtil.drawRobot(overlay, controller.targetState.pose)
        }

        overlay.setStroke("red")
        DashboardUtil.drawRobot(overlay, position)
        DashboardUtil.drawPoseHistory(overlay, poseHistory)

        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }

    override val displacement: Double
        get() {
            require(controller != null) { "Tried to access displacement with null controller"}

            return controller!!.targetState.displacement
        }

    override val isAtDestination: Boolean
        get() {
            require(controller != null) { "Tried to access destination status with null controller" }

            return controller!!.isWithinTolerance
        }

    override fun beginFollow(trajectory: Trajectory) {
        controller = HoloTrajectoryController(trajectory)
    }

    override fun isActualTrajectory(trajectory: Trajectory): Boolean {
        if(controller == null) {
            return false
        }

        return controller!!.trajectory == trajectory
    }
}