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

    return targetVelRobot + Twist2dDual.const(
        Twist2d(
            KPosX * posError.translation.x,
            KPosY * posError.translation.y,
            KPosR * posError.rotation.log()
        ),
        2 // Velocity and acceleration command
    )
}

class MecanumDrive(hardwareMap: HardwareMap) {
    val motorFL = hardwareMap.get(DcMotorEx::class.java, "MotorFL")
    val motorFR = hardwareMap.get(DcMotorEx::class.java, "MotorFR")
    val motorBL = hardwareMap.get(DcMotorEx::class.java, "MotorBL")
    val motorBR = hardwareMap.get(DcMotorEx::class.java, "MotorBR")
    val motors = listOf(motorFL, motorFR, motorBL, motorBR)

    val voltageSensor = hardwareMap.voltageSensor.first()

    private val localizer = Holo3WheelLocalizer(hardwareMap)

    var position = Pose2d(0.0, 0.0, 0.0)
        private set

    private val poseHistory = ArrayList<Pose2d>(1000)

    private val watch = Stopwatch()

    init {
        motors.forEach { motor ->
            motor.motorType = motor.motorType.clone().also {
                it.achieveableMaxRPMFraction = 1.0
            }

            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        motorFL.direction = DcMotorSimple.Direction.REVERSE
        motorBL.direction = DcMotorSimple.Direction.REVERSE
    }

    fun resetPower() {
        motors.forEach { it.power = 0.0 }
    }

    fun applyPower(power: Twist2d) {
        val vel = MecanumKinematics.inverse(Twist2dDual.const(power), A, B)

        motorFL.power = vel.frontLeft.value
        motorFR.power = vel.frontRight.value
        motorBL.power = vel.backLeft.value
        motorBR.power = vel.backRight.value
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

        if(poseHistory.size > 1000){
            poseHistory.removeAt(0)
        }

        val packet = TelemetryPacket()

        packet.addLine("Update Rate: ${1.0 / watch.sample()}")

        val overlay = packet.fieldOverlay()

        overlay.setStroke("red")

        DashboardUtil.drawRobot(overlay, position)
        DashboardUtil.drawPoseHistory(overlay, poseHistory)

        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }
}