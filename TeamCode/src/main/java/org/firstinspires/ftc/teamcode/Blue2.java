package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@Autonomous(name = "autoBlue2")

public class Blue2 extends driveToCol {
    OpenGLMatrix lastLocation = null;
    double distanceToGo = 0;
    VuforiaLocalizer vuforia;
    Servo servoGlipSides;
    Servo servoGlipHand;
    Servo servoGlipBring;
    Servo servoSensor;
    Servo servoArm;
    ColorSensor sensorColor;
    ColorSensor sensorGlip;
    BNO055IMU imu;
    Orientation angles, angles2;
    ModernRoboticsI2cRangeSensor range;
    DcMotor[][] dcmotor = new DcMotor[5][4];
    double motorPower = 0;
    int modeTurn; // TURN PER MODE
    int S;// STRAFE PER MODE
    int D;// DRIVE DISTANCE PER MODE
    int modeOri;// ORIENTATION PER MODE
    int mode = 3;
    RelicRecoveryVuMark v;
    DcMotor[] intakeDC = new DcMotor[2];
}