package org.firstinspires.ftc.teamcode.dashboard;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConfig {
    public static double A = 0.25;
    public static double B = 0.25;

    public static double Ks = 0.8;
    public static double Kv = 7.4;
    public static double Ka = 0.8;

    public static double KPosX = 5;
    public static double KPosY = 5;
    public static double KPosR = 3;

    public static double AdmissibleDistance = 0.05;
    public static double AdmissibleAngleDeg = 5;

    public static double DriveTVel = 1.25;
    public static double DriveRVelDeg = 150;
}
