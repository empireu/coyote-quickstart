package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import empireu.coyote.Odometry
import empireu.coyote.Twist2dIncr
import org.firstinspires.ftc.teamcode.dashboard.LocalizerConfig
import org.firstinspires.ftc.teamcode.dashboard.LocalizerConfig.TPR
import org.firstinspires.ftc.teamcode.dashboard.LocalizerConfig.WhlR
import java.lang.Math.PI

class Encoder(private val motor: DcMotorEx, var sign: Int = 1) {
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

class WheelEncoder(val encoder: Encoder, val wheelRadius: Double, val tpr: Int) {
    private fun convertTicks(ticks: Double) =
        wheelRadius * 2.0 * PI * ticks / tpr.toDouble()

    fun readIncr(): Double = convertTicks(encoder.readIncr().toDouble())
}

interface ILocalizer {
    fun readIncr(): Twist2dIncr
}

class Holo3WheelLocalizer(hwMap: HardwareMap) : ILocalizer {
    val lEnc = WheelEncoder(Encoder(hwMap, "MotorBR"), WhlR, TPR)
    val rEnc = WheelEncoder(Encoder(hwMap, "MotorFL"), WhlR, TPR)
    val cEnc = WheelEncoder(Encoder(hwMap, "MotorFR").reversed(), WhlR, TPR)

    override fun readIncr() = Odometry.holo3WheelIncr(
        lIncr = lEnc.readIncr(),
        rIncr = rEnc.readIncr(),
        cIncr = cEnc.readIncr(),
        lY = LocalizerConfig.LY,
        rY = LocalizerConfig.RY,
        cX = LocalizerConfig.CX
    )
}