package org.firstinspires.ftc.teamcode;
public class CONSTANTS {

//check commit!

    public static final double fwSpeed = 1090;
    public static final double fwFarSpeed = 1090;
    public static final double fwNearSpeed = 850;
    public static final double autoFwSpeed = 1120;
    public static final double autoFwSpeed2 = 1120;
    public static final double autoCloseFwSpeed = 870;
    public static final double intakeSpeed = 0.9;
    public static final double dpadSpeed = 0.4;
    public static final double wackUp = 0.48;
    public static final double wackDown = 0.23;
    public static final double llServoMin = 0.05;
    public static final double llServoMax = 1;
    //    static final double[] suzani = {0.1, 0.48, .83};
    public static final double[] suzani = {0.19, 0.56, 0.93};

    //    static final double[] suzano = {0.67, 1, 0.3};
    public static final double[] suzano = {0.75, 0.01, 0.38};
    public static final double[] flicksDown = {0.1, 0.1, 0.1};
    public static final double[] flicksUp = {0.5, 0.5, 0.5};

    //    0.74  0.36  0
//    0.16  0.53  0.91
    public static final double searchSpeed = 0.02;
    public static double[] purpleBall = {145, 175, 250, 190};
    public static double[] greenBall = {55, 142, 113, 103};
    //    BLUE
    public static final double blueX = 1.0;
    public static final double blueY = -0.5;
    public static final double blueYaw = Math.toRadians(-69);
    public static final double blueInX = 1.5;
    public static final double blueInY = -1.5;
    public static final double blueInYaw = Math.toRadians(90);
    //    RED
    public static final double redX = 1.0;
    public static final double redY = -0.5;
    public static final double redYaw = Math.toRadians(-115);
    public static final double redInX = 1.5;
    public static final double redInY = -1.5;
    public static final double redInYaw = Math.toRadians(90);

    //    Limelight offsets
// Camera offset relative to robot CENTER, in meters
    public static final double CAMERA_OFFSET_X = -0.026; // forward (+), backward (-)
    public static final double CAMERA_OFFSET_Y = 0.163; // left (+), right (-)
    // Servo angle limits → convert servo position to angle

    public static final double SERVO_CENTER_POS = 0.51;  // Forward = 0 rad
    public static final double SERVO_MIN_POS = 0.15;
    public static final double SERVO_MAX_POS = 0.80;

    // Total useful span (example): 260 degrees = 4.537 rad
    public static final double SERVO_TOTAL_ANGLE = Math.toRadians(260);

    //    AUTO
    public static final double AutoSlow = 400;
    public static final double AutoFast = 1200;
    public static final double AutoTurnSlow = 400;
    public static final double AutoTurnFast = 1000;
    public static final double posTol = 1;
    public static final double yawTol = Math.toRadians(0.5);
    //    BLUE
    public static final double blueShootX = 6;
    public static final double blueShootY = 2;
    public static final double blueShootYaw = -66;
    public static final double blueShootYaw2 = -70;
    public static final double blueIntake1X = 26;
    public static final double blueIntakeY = 12;
    public static final double blueIntakeYaw = 90;
    public static final double blueLeaveX = 15;
    public static final double blueLeaveY = 5;
    public static final double blueLeaveYaw = 0;
    //    RED
    public static final double redShootX = 6;
    public static final double redShootY = -2;
    public static final double redShootYaw = -109;
    public static final double redShootYaw2 = -115;
    public static final double redIntake1X = 26;
    public static final double redIntakeY = -12;
    public static final double redIntakeYaw = -90;
    public static final double redLeaveX = 13;
    public static final double redLeaveY = -4;
    public static final double redLeaveYaw = 0;
}
