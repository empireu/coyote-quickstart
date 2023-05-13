package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import empireu.coyote.MecanumKinematics
import empireu.coyote.Pose2d
import empireu.coyote.Twist2d
import empireu.coyote.Twist2dDual
import org.firstinspires.ftc.teamcode.dashboard.DriveConfig
import org.firstinspires.ftc.teamcode.dashboard.LocalizerConfig
import kotlin.math.PI

@TeleOp(name = "Motor Test")
class MotorTestOpMode: LinearOpMode() {
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap)

        waitForStart()
        drive.motors.forEach {
            it.power = 0.1
            sleep(2500)
            it.power = 0.0
        }
    }
}

@TeleOp(name = "Manual Control")
class ManualOpMode : LinearOpMode() {
    override fun runOpMode() {
        FtcDashboard.getInstance().telemetryTransmissionInterval = 10
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val drive = MecanumDrive(hardwareMap)
        val localizer = Holo3WheelLocalizer(hardwareMap)

        val lynxModules = hardwareMap.getAll(LynxModule::class.java)

        lynxModules.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        var pose = Pose2d(0.0, 0.0, 0.0)

        val watch = Stopwatch()

        val ftAvg = Average(25)

        val history = ArrayList<Pose2d>()

        while (!isStopRequested){
            lynxModules.forEach { it.clearBulkCache() }

            val elapsed = watch.sample()

            if(elapsed > 0) {
                telemetry.addData("Update Rate", 1.0 / ftAvg.update(elapsed))
            }

            drive.applyPower(
                Twist2d(
                    gamepad1.left_stick_y.toDouble(),
                    gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble()
                )
            )

            //telemetry.addData("l", localizer.lEnc.readPosition())
            //telemetry.addData("r", localizer.rEnc.readPosition())
            //telemetry.addData("c", localizer.cEnc.readPosition())

            pose += localizer.readIncr()
            history.add(pose)

            //telemetry.addData("Pose", pose)

            //telemetry.update()

            val packet = TelemetryPacket()
            val overlay = packet.fieldOverlay()
            DashboardUtil.drawRobot(overlay, pose)
            DashboardUtil.drawPoseHistory(overlay, history)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)

            if(history.size > 1000){
                history.removeAt(0)
            }
        }
    }
}

@TeleOp(name = "Odometry Tuner")
class OdometryTunerOpMode : LinearOpMode() {
    override fun runOpMode() {
        telemetry = FtcDashboard.getInstance().telemetry

        val drive = MecanumDrive(hardwareMap)
        val localizer = Holo3WheelLocalizer(hardwareMap)

        waitForStart()

        var l = 0.0
        var r = 0.0
        var c = 0.0

        val angularDisp = 2.0 * PI * LocalizerConfig.TuningTurns

        while (opModeIsActive()) {
            drive.applyPower(
                Twist2d(
                    0.0,
                    0.0,
                    -gamepad1.right_stick_x.toDouble() * 4
                )
            )

            l += localizer.lEnc.readDistIncr()
            r += localizer.rEnc.readDistIncr()
            c += localizer.cEnc.readDistIncr()

            telemetry.addData("left distance", l / angularDisp)
            telemetry.addData("right distance", r / angularDisp)
            telemetry.addData("center distance", c / angularDisp)
            telemetry.update()
        }

        // Encoders are reading 0 around the time of stopping. I kept the calculation inside the loop because of this.
        // Why is this happening, and how do I fix it?
        // I am clearing the readout here so as to not cause confusion.
        telemetry.clear()
    }
}
