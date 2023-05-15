package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import empireu.coyote.*
import org.firstinspires.ftc.teamcode.dashboard.CoyoteConfig.*
import org.firstinspires.ftc.teamcode.dashboard.LocalizerConfig
import org.firstinspires.ftc.teamcode.temp.ArmSystem
import java.io.BufferedInputStream
import java.io.File
import java.io.FileOutputStream
import java.lang.Integer.parseInt
import java.net.URL
import kotlin.math.PI

@TeleOp(name = "Motor Test", group = "Tuning")
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

        waitForStart()
        while (!isStopRequested){
            lynxModules.forEach { it.clearBulkCache() }

            drive.applyDrivePower(
                Twist2d(
                    gamepad1.left_stick_y.toDouble(),
                    gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble()
                )
            )

            drive.update()
        }
    }
}

@TeleOp(name = "Odometry Tuner", group = "Tuning")
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
            drive.applyDrivePower(
                Twist2d(
                    0.0,
                    0.0,
                    -gamepad1.right_stick_x.toDouble()
                )
            )

            l += localizer.lEnc.readIncr()
            r += localizer.rEnc.readIncr()
            c += localizer.cEnc.readIncr()

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
            sleep(1000)
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
        val dashboard = FtcDashboard.getInstance()
        dashboard.telemetryTransmissionInterval = 10
        telemetry.isAutoClear = true
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        val editorProject = loadRobotCoyoteProject()

        val savedNodeProject = editorProject.NodeProjects[NodeProjectName]
            ?: error("Failed to get node project $NodeProjectName")

        val drive = MecanumDrive(hardwareMap)
        val node: BehaviorNode

        try {
            node = loadNodeProject(
                savedNodeProject.RootNodes,
                BehaviorMapBuilder().also { b ->
                    b.add("Sequence", { ctx -> BehaviorSequenceNode(ctx.name, ctx.runOnce, ctx.childNodes) })
                    b.add("Selector", { ctx -> BehaviorSelectorNode(ctx.name, ctx.runOnce, ctx.childNodes) })
                    b.add("Success", { ctx -> BehaviorResultNode(ctx.name, ctx.runOnce, ctx.one(), BehaviorStatus.Success) })
                    b.add("Parallel", { ctx -> BehaviorParallelNode(ctx.name, ctx.runOnce, ctx.childNodes) })
                    b.add("Repeat", { ctx -> BehaviorRepeatNode(ctx.name, ctx.runOnce, parseInt(ctx.savedData), ctx.one()) })

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

                    b.add("Motion", { ctx -> BehaviorMotionNode(ctx, editorProject, drive) })

                    registerNodes(b)
                }.build()
            ).behaviors
                .filter { it.root.name == EntryNodeName }
                .also {
                    if(it.isEmpty()) {
                        error("Failed to find root node $EntryNodeName")
                    }

                    if(it.size != 1) {
                        error("Ambiguous root node $EntryNodeName")
                    }
                }
                .first()
                .root
        }
        catch (t: Throwable) {
            telemetry.addData("ERROR", t.fillInStackTrace())
            telemetry.update()
            sleep(10000)
            return
        }

        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        lynxModules.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        val context = BehaviorContext()
        waitForStart()

        fun needsStop() = isStopRequested || gamepad1.left_bumper || gamepad1.right_bumper || gamepad2.left_bumper || gamepad2.right_bumper

        try {
            while (!needsStop()) {
                lynxModules.forEach { it.clearBulkCache() }

                val status = node.getStatus(context)

                if(status != BehaviorStatus.Running){
                    break
                }

                drive.update()
            }
        }
        finally {
            drive.resetPower()
        }
    }

    private fun registerNodes(builder: BehaviorMapBuilder) {
        builder.add("Arm", { ctx ->
            SystemNode(ctx) { ArmSystem(it, hardwareMap) }
        })
    }
}
