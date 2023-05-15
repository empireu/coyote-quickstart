package org.firstinspires.ftc.teamcode.temp

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import empireu.coyote.BehaviorStatus
import org.firstinspires.ftc.teamcode.ICompositeNode
import org.firstinspires.ftc.teamcode.ISystem
import org.firstinspires.ftc.teamcode.temp.ArmConfig.*
import kotlin.math.abs

/**
 * Example arm, integrated into the Behavior Tree system.
 * */
class ArmSystem(composite: ICompositeNode, hardwareMap: HardwareMap) : ISystem {
    enum class ArmPreset {
        Pickup, Carry, Place
    }

    /**
     * Maps to the editor YAML definition.
     * */
    data class ArmStorage(
        /**
         * Maps to the flag *closed*.
         * It indicates if the claw should be closed or open.
         * */
        val closed: Boolean,

        /**
         * Maps to the enum *preset*.
         * It contains the desired state of the arm assembly.
         * */
        val preset: ArmPreset
    )

    private val storage = composite.load(ArmStorage::class)

    private val jointMotor: DcMotor = hardwareMap.dcMotor.get("MotorJoint")
    private val elevatorMotor: DcMotor = hardwareMap.dcMotor.get("MotorElevator")
    private val rotatorServo: Servo = hardwareMap.servo.get("ServoRotator")
    private val clawServo1: Servo = hardwareMap.servo.get("ServoClaw1")
    private val clawServo2: Servo = hardwareMap.servo.get("ServoClaw2")

    private val configTarget: ArmOption
    private val claw1Target: Double
    private val claw2Target: Double

    // Systems are created as needed by the System Node. This is the other way to handle execution state storage.
    // It basically wraps the context and stores this system instance. It gives us the freedom to store the variables we need right in the class.
    private var clawStarted = false

    init {
        jointMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        elevatorMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        configTarget = when(storage.preset) {
            ArmPreset.Pickup -> { Pickup }
            ArmPreset.Carry -> { Carry }
            ArmPreset.Place -> { Place }
        }

        if(storage.closed){
            claw1Target = Claw1Closed
            claw2Target = Claw2Closed
        }
        else{
            claw1Target = Claw1Open
            claw2Target = Claw2Open
        }
    }

    // Instead of setting those up in the loop, we can set them here, assuming this node was correctly
    // defined as non-parallel in the editor definition. Of course, this will not handle cases such as the hardware getting accessed from somewhere else.
    // Setting them up once is useful because it doesn't issue any extra hardware calls (that could impede the update speed of the program)
    // Performance is important because this node could be running in parallel with time-sensitive processes (e.g. localization while driving)

    override fun initialize() {
        jointMotor.targetPosition = configTarget.Joint
        elevatorMotor.targetPosition = configTarget.Elevator

        jointMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        elevatorMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        jointMotor.power = JointSpeed
        elevatorMotor.power = ElevatorSpeed

        rotatorServo.position = configTarget.Rotator
    }

    override fun update(): BehaviorStatus {
        if(
            abs(configTarget.Elevator - elevatorMotor.currentPosition) > ElevatorTolerance ||
            abs(configTarget.Joint - jointMotor.currentPosition) > JointTolerance ||
            abs(configTarget.Rotator - rotatorServo.position) > RotatorTolerance)
        {
            // Wait for claw assembly to reach target before closing/opening claw
            return BehaviorStatus.Running
        }

        // The elevator, joint and rotator are in the desired configuration.
        // We can now close/open the claw:
        if(!clawStarted){
            clawStarted = true // Again, prevent extra hardware calls.
            clawServo1.position = claw1Target
            clawServo2.position = claw2Target
        }

        if(abs(claw1Target - clawServo1.position) < ClawTolerance && abs(claw2Target - clawServo2.position) < ClawTolerance) {
            return BehaviorStatus.Success
        }

        return BehaviorStatus.Running
    }
}