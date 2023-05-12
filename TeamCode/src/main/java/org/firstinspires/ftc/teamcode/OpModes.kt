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

            val velocities = MecanumKinematics.inverse(
                Twist2dDual.const(
                    Twist2d(
                        gamepad1.left_stick_y.toDouble(),
                        gamepad1.left_stick_x.toDouble(),
                        -gamepad1.right_stick_x.toDouble()
                    )
                ),
                DriveConfig.A,
                DriveConfig.B
            )

            drive.applyVelocities(velocities)

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
