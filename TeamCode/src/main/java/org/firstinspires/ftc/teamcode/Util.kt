package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.canvas.Canvas
import empireu.coyote.Pose2d
import empireu.coyote.Vector2d
import empireu.coyote.nonZero

import kotlin.math.sqrt

fun meterToInch(meters: Double): Double = meters * 39.3700787

fun Vector2d.toInches(): Vector2d {
    return this * Vector2d(meterToInch(1.0))
}

fun Pose2d.toInches(): Pose2d {
    return Pose2d(this.translation.toInches(), this.rotation)
}

/**
 * Stopwatch using [System.nanoTime].
 * */
class Stopwatch {
    private var samplingTimestamp = System.nanoTime()
    private var initialTimestamp = samplingTimestamp

    val total get() = convert(System.nanoTime() - initialTimestamp)

    /**
     * Resets the value returned by [total].
     * */
    fun resetTotal() {
        initialTimestamp = System.nanoTime()
    }

    /**
     * Samples the value and resets the timer. This, however, will not affect values
     * returned by [total].
     *
     * @see resetTotal
     * */
    fun sample(): Double {
        val current = System.nanoTime()
        val elapsedNanoseconds = current - samplingTimestamp
        samplingTimestamp = current
        return convert(elapsedNanoseconds)
    }

    companion object {
        private fun convert(elapsedNanoseconds: Long): Double {
            return elapsedNanoseconds / 1000000000.0
        }
    }
}

object DashboardUtil {
    private const val ROBOT_RADIUS = 9.0 // in

    fun drawPoseHistory(canvas: Canvas, poseHistory: List<Pose2d>) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        for (i in poseHistory.indices) {
            val pose = poseHistory[i].toInches()
            xPoints[i] = pose.translation.x
            yPoints[i] = pose.translation.y
        }

        canvas.strokePolyline(xPoints, yPoints)
    }

    fun drawRobot(canvas: Canvas, p: Pose2d) {
        val pose = p.toInches()
        canvas.strokeCircle(pose.translation.x, pose.translation.y, ROBOT_RADIUS)

        val v = pose.rotation.direction * ROBOT_RADIUS
        val x1 = pose.translation.x + v.x / 2
        val y1 = pose.translation.y + v.y / 2
        val x2 = pose.translation.x + v.x
        val y2 = pose.translation.y + v.y

        canvas.strokeLine(x1, y1, x2, y2)
    }

    fun drawLinePointing(canvas: Canvas, start: Vector2d, line: Vector2d) {
        val startInch = start.toInches()
        val lineInch = line.toInches()

        canvas.strokeLine(
            startInch.x,
            startInch.y,
            startInch.x + lineInch.x,
            startInch.y + lineInch.y
        )
    }
}

class Average(val sampleCount: Int) {
    private val storage = ArrayList<Double>(sampleCount)

    fun add(value: Double) {
        storage.add(value)

        if(storage.size > sampleCount){
            storage.removeAt(0)
        }
    }

    fun compute() = storage.sum() / storage.size.toDouble().nonZero()

    fun update(value: Double): Double {
        add(value)

        return compute()
    }
}