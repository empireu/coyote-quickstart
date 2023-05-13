package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import empireu.coyote.*
import org.firstinspires.ftc.teamcode.dashboard.CoyoteConfig.*
import org.firstinspires.ftc.teamcode.dashboard.LocalizerConfig
import java.io.BufferedInputStream
import java.io.File
import java.io.FileOutputStream
import java.net.URL
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
        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        lynxModules.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        while (!isStopRequested){
            lynxModules.forEach { it.clearBulkCache() }

            /*drive.applyPower(
                Twist2d(
                    gamepad1.left_stick_y.toDouble(),
                    gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble()
                )
            )*/

            drive.applyWheelCommand(
                holoCommand(
                    Pose2dDual(
                        Vector2dDual(
                            Dual.of(drive.position.translation.x, gamepad1.left_stick_y.toDouble(), 0.0),
                            Dual.of(drive.position.translation.y, gamepad1.left_stick_x.toDouble(), 0.0)
                        ),
                        Rotation2dDual.exp(
                            Dual.of(drive.position.rotation.log(), -gamepad1.right_stick_x.toDouble(), 0.0)
                        )
                    ),
                    drive.position
                )
            )

            drive.update()
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

@TeleOp(name = "Coyote Download", group = "Coyote")
class CoyoteDownload : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val writer: FileOutputStream

        try {
            if(File(robotCoyotePath).exists()){
                File(robotCoyotePath).delete()
            }

            File(robotCoyotePath).createNewFile()

            writer = FileOutputStream(robotCoyotePath)
        } catch (t: Throwable){
            telemetry.addData("Failed to open local storage", t)
            telemetry.update()
            sleep(30000)
            return
        }

        telemetry.addData("Target", robotCoyotePath)
        telemetry.update()

        waitForStart()

        try {
            val networkStream = BufferedInputStream(URL(CoyoteDownloadUrl).openStream())

            val buffer = ByteArray(1024)
            var read: Int
            var total = 0

            while (networkStream.read(buffer, 0, 1024).also { read = it } != -1) {
                writer.write(buffer, 0, read)
                total += read
            }

            networkStream.close()
            writer.close()

            telemetry.addData("Download size: ", total)
            telemetry.update()
        }
        catch (t: Throwable){
            telemetry.addData("Network Error: ", t)
            telemetry.update()
            sleep(30000)
        }
    }
}

@TeleOp(name = "Coyote", group = "Coyote")
class CoyoteOpMode : LinearOpMode() {
    override fun runOpMode() {
        val editorProject = loadRobotCoyoteProject()

        val savedNodeProject = editorProject.NodeProjects[NodeProjectName]
            ?: error("Failed to get node project $NodeProjectName")

        val rootJsonNode = savedNodeProject
            .RootNodes.filter { it.Name == EntryNodeName }
            .also {
                if(it.isEmpty()) {
                    error("Failed to find root node $EntryNodeName")
                }

                if(it.size != 1) {
                    error("Ambiguous root node $EntryNodeName")
                }
            }.first()

        val nodeProject = loadNodeProject(
            savedNodeProject.RootNodes,
            BehaviorMapBuilder().also { b ->
                b.add("Sequence", { ctx -> BehaviorSequenceNode(ctx.name, ctx.runOnce, ctx.childNodes) })
                b.add("Selector", { ctx -> BehaviorSelectorNode(ctx.name, ctx.runOnce, ctx.childNodes) })
                b.add("Success", { ctx -> BehaviorResultNode(ctx.name, ctx.runOnce, ctx.one(), BehaviorStatus.Success) })
                b.add("Parallel", { ctx -> BehaviorParallelNode(ctx.name, ctx.runOnce, ctx.childNodes) })

                b.add(
                    "Call",
                    { ctx -> BehaviorCallNode(ctx.name, ctx.runOnce) },

                    // The call nodes need a second pass (to search for the target node):
                    { ctx ->
                        val target = ctx.project.behaviors.firstOrNull { it.root.name == ctx.createContext.savedData }
                            ?: error("Failed to bind call node")

                        ctx.node.bind(target.root)
                    }
                )
            }.build()
        )
    }
}
