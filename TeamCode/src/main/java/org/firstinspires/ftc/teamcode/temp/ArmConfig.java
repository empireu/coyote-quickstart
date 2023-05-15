package org.firstinspires.ftc.teamcode.temp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConfig {
    public static class ArmOption{
        public int Joint;
        public int Elevator;
        public double Rotator;

        public ArmOption(int joint, int elevator, double rotator){
            Joint = joint;
            Elevator = elevator;
            Rotator = rotator;
        }
    }

    // My elevator string broke. I don't have any replacements for now.
    public static ArmOption Pickup = new ArmOption(230, 0, 0.8);
    public static ArmOption Carry = new ArmOption(800, 1300, 1);
    public static ArmOption Place = new ArmOption(800, 1100, 1);

    public static double ElevatorSpeed = 0.5;
    public static double JointSpeed = 0.5;

    public static double Claw1Closed = 0.0;
    public static double Claw2Closed = 1.0;
    public static double Claw1Open = 0.5;
    public static double Claw2Open = 0.75;

    public static double ElevatorTolerance = 30;
    public static double JointTolerance = 50;
    public static double RotatorTolerance = 0.05;
    public static double ClawTolerance = 0.05;
}
