package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import empireu.coyote.*
import org.firstinspires.ftc.teamcode.dashboard.DriveConfig.*
import org.firstinspires.ftc.teamcode.dashboard.LocalizerConfig.*
import kotlin.math.absoluteValue

class Encoder(val motor: DcMotorEx, var sign: Int = 1) {
    constructor(map: HardwareMap, name: String) : this(
        map.get(DcMotorEx::class.java, name)
            ?: error("Failed to get encoder $name")
    )

    fun reversed(): Encoder {
        return Encoder(motor, -sign)
    }

    private var lastPos: Int

    init {
        lastPos = readPosition()
    }

    fun readPosition() = motor.currentPosition * sign

    fun readIncr(): Int {
        val current = readPosition()
        val increment = current - lastPos
        lastPos = current

        return increment
    }
}

fun convertTicks(ticks: Double) = WheelRadius * 2.0 * Math.PI * ticks / TPR
fun Encoder.readDistIncr() = convertTicks(this.readIncr().toDouble())

class Holo3WheelLocalizer(hardwareMap: HardwareMap) {
    val lEnc = Encoder(hardwareMap, "MotorBR")
    val rEnc = Encoder(hardwareMap, "MotorFL")
    val cEnc = Encoder(hardwareMap, "MotorFR").reversed()

    fun readIncr() = Odometry.holo3WheelIncr(
        lIncr = lEnc.readDistIncr(),
        rIncr = rEnc.readDistIncr(),
        cIncr = cEnc.readDistIncr(),
        lY = LY,
        rY = RY,
        cX = CX
    )
}

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
    val motorFL = hardwareMap.get(DcMotorEx::class.java, "MotorFL")
    val motorFR = hardwareMap.get(DcMotorEx::class.java, "MotorFR")
    val motorBL = hardwareMap.get(DcMotorEx::class.java, "MotorBL")
    val motorBR = hardwareMap.get(DcMotorEx::class.java, "MotorBR")
    val motors = listOf(motorFL, motorFR, motorBL, motorBR)

    val voltageSensor = hardwareMap.voltageSensor.first()

    private val localizer: Holo3WheelLocalizer

    var position = Pose2d(0.0, 0.0, 0.0)
        private set

    private val poseHistory = ArrayList<Pose2d>(1000)

    private val watch = Stopwatch()

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

    fun resetPower() {
        motors.forEach { it.power = 0.0 }
    }

    fun applyDrivePower(power: Twist2d) {
        if(power.approxEqs(Twist2d(0.0, 0.0, 0.0), 10e-5)){
            resetPower()

            return
        }

        applyWheelCommand(
            Twist2dDual.const(
                Twist2d(
                    power.trVelocity * DriveTVel,
                    power.rotVelocity * Math.toRadians(DriveRVelDeg)
                ), 2
            )
        )
    }

    fun applyWheelCommand(velRobot: Twist2dDual) {
        val vel = MecanumKinematics.inverse(velRobot, A, B)
        val voltage = voltageSensor.voltage
        motorFL.power = vCompFeedForwardDual(vel.frontLeft, voltage)
        motorFR.power = vCompFeedForwardDual(vel.frontRight, voltage)
        motorBL.power = vCompFeedForwardDual(vel.backLeft, voltage)
        motorBR.power = vCompFeedForwardDual(vel.backRight, voltage)
    }

    fun update() {
        position += localizer.readIncr()

        poseHistory.add(position)

        if(poseHistory.size > 500){
            poseHistory.removeAt(0)
        }

        val packet = TelemetryPacket()
        packet.addLine("Update Rate: ${1.0 / watch.sample()}")
        val overlay = packet.fieldOverlay()

        val controller = this.controller

        if(controller != null) {
            applyWheelCommand(
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