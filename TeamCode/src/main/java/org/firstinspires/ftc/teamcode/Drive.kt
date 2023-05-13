package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import empireu.coyote.MecanumKinematics
import empireu.coyote.Odometry
import empireu.coyote.Twist2d
import empireu.coyote.Twist2dDual
import org.firstinspires.ftc.teamcode.dashboard.DriveConfig
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

class MecanumDrive(hardwareMap: HardwareMap) {
    val motorFL: DcMotorEx
    val motorFR: DcMotorEx
    val motorBL: DcMotorEx
    val motorBR: DcMotorEx
    val motors: List<DcMotor>

    init {
        motorFL = hardwareMap.get(DcMotorEx::class.java, "MotorFL")
        motorFR = hardwareMap.get(DcMotorEx::class.java, "MotorFR")
        motorBL = hardwareMap.get(DcMotorEx::class.java, "MotorBL")
        motorBR = hardwareMap.get(DcMotorEx::class.java, "MotorBR")

        motors = listOf(motorFL, motorFR, motorBL, motorBR)

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
        val vel = MecanumKinematics.inverse(
            Twist2dDual.const(power),
            DriveConfig.A,
            DriveConfig.B
        )

        motorFL.power = vel.frontLeft.value
        motorFR.power = vel.frontRight.value
        motorBL.power = vel.backLeft.value
        motorBR.power = vel.backRight.value
    }


}