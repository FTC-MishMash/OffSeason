package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
@Disabled

//@Autonomous(name = "Final Red 1")
public class AutoTest extends LinearOpMode  {
    HardwareMap aw=null;
    OpenGLMatrix lastLocation = null;
    ElapsedTime runtime = new ElapsedTime();
    double distanceToGo = 0;
    VuforiaLocalizer vuforia;
    Servo servoGlipSides;
    DistanceSensor sensorColorDistanse;
    Servo servoGlipBring;
    Servo servoSensor;
    Servo servoArm;
    ColorSensor sensorColor;
    ColorSensor sensorColorBack;
    ColorSensor sensorGlip1;
    ColorSensor sensorGlip2;
    BNO055IMU imu;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    double cmRound = 29;
    double tixRound = 700;
    Orientation angles, angles2;
    ModernRoboticsI2cRangeSensor range;
    DcMotor[][] dcmotor = new DcMotor[5][4];
    int mode = 1;
    DistanceSensor sensorDistance;
    double motorPower = 0;
    int modeTurn; // TURN PER MODE
    int S;// STRAFE PER MODE
    int D;// DRIVE DISTANCE PER MODE
    int modeOri;// ORIENTATION PER MODE
    RelicRecoveryVuMark v;
    DcMotor[] intakeDC = new DcMotor[2];
    int nSleep = 100;
    final double SCALE_FACTOR = 255;
    int bColor = 0;
    I2cDeviceSynch pixyCam;

    double x1, y1, width1, height1, numObjects1;
    double x2, y2, width2, height2, numObjects2;
    AnalogInput pot;
    HardwareMap hwMap=null;
    byte[] pixyData1, pixyData2;


    public void init(HardwareMap aw){
        hwMap=aw;
        pixyCam = hwMap.get(I2cDeviceSynch.class, "pixy");
        range = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sRange");
        imu = hwMap.get(BNO055IMU.class, "imu");
        dcmotor[0][1] = hwMap.get(DcMotor.class, "right1");
        dcmotor[1][1] = hwMap.get(DcMotor.class, "right2");
        dcmotor[0][0] = hwMap.get(DcMotor.class, "left1");
        dcmotor[1][0] = hwMap.get(DcMotor.class, "left2");
        intakeDC[0] = hwMap.get(DcMotor.class, "g1");
        intakeDC[1] = hwMap.get(DcMotor.class, "g2");
        sensorColor = hwMap.get(ColorSensor.class, "sensorColor");
        sensorColorBack = hwMap.get(ColorSensor.class, "sensorColorRight");
//        dcmotor[0][1].setDirection(DcMotorSimple.Direction.REVERSE);
//        dcmotor[1][1].setDirection(DcMotorSimple.Direction.REVERSE);
        servoGlipSides = hwMap.get(Servo.class, "servoSides");
        //servoGlipHand = hardwareMap.get(Servo.class, "servoHand");
        servoGlipBring = hwMap.get(Servo.class, "servoBring");
        servoArm = hwMap.get(Servo.class, "servoArm");
        servoSensor = hwMap.get(Servo.class, "servoSensor");
        sensorColorDistanse = (DistanceSensor) hwMap.get(DistanceSensor.class, "upperColorDistanse");
        pot = hwMap.get(AnalogInput.class, "pot");
        dcmotor[0][0].setDirection(DcMotorSimple.Direction.REVERSE);
        dcmotor[0][1].setDirection(DcMotorSimple.Direction.FORWARD);
        dcmotor[1][0].setDirection(DcMotorSimple.Direction.REVERSE);
        dcmotor[1][1].setDirection(DcMotorSimple.Direction.FORWARD);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    //    telemetry.addData("IN SERVO init", servoSensor.getPosition());
        //telemetry.update();
        switch (mode) {
            case 1: {//red 1
                modeTurn = 90;
                bColor = 0;
                break;
            }
            case 2: {//red 2
                modeTurn = 180;
                bColor = 0;
                break;
            }
            case 3: {//blue 1
                modeTurn = 90;
                bColor = 1;
                break;
            }
            case 4: {//blue 2
                modeTurn = 0;
                bColor = 1;

                // set the mode the robot use in the autumonus
            }
        }


        double power = -0.3;
        double pos = 0;
        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
       // final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        runtime.reset();
        servoGlipSides.setPosition(0);
        int colVu;
        colVu = GetColoumn();
        //elemetry.addData("Vufuria", colVu);
       // telemetry.update();


//        waitForStart();
//        knockBall(0);
//        Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
//                (int) (sensorColorBack.green() * SCALE_FACTOR),
//                (int) (sensorColorBack.blue() * SCALE_FACTOR),
//                hsvValues);
//        telemetry.addData("hue", hsvValues[0]);
//        telemetry.update();
////        runtime.reset();
//        telemetry.addLine("getDownWithColor");
//        telemetry.update();
//        getDownWithColor(-0.35, bColor);
//        sleep(nSleep);
//        driveByColor(0, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, -0.34);
//        sleep(nSleep);
//        telemetry.addLine("ENCODERS");
//        telemetry.update();
//        sleep(1000);
//        NewdriveRobotEncoder(10, 180, 0.25, 0);
//        sleep(nSleep);
//        Turn(modeTurn - 10);
//        telemetry.addLine("DriveByDistance");
//        telemetry.update();
//        DriveByDistance(20, -0.25, range, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//        sleep(nSleep);
//        telemetry.addLine("Turn Vufuria");
//        telemetry.update();
//        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//        if (colVu == 1) {
//            Turn(-45);
//        }
//        //if col Vu === 2 the robot dont need to turn
//        if (colVu == 3) {
//            Turn(45);
//        }
//        sleep(nSleep);
//        telemetry.addLine("scoreGlip");
//        telemetry.update();
//        scoreGlip(modeTurn);
//        sleep(nSleep);
//        Turn(angle - imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//        sleep(nSleep);
//        telemetry.addLine("drive By Color");
//        telemetry.update();
//        intakeDC[0].setPower(-0.8);
//        intakeDC[1].setPower(0.8);
//        driveByColor(0, modeTurn, -0.3);
//        sleep(nSleep);
//        telemetry.addLine("get More Glyph");
//        telemetry.update();
//        getMoreGlyph1(-0.55, 0, 80);
//        sleep(nSleep);
//        telemetry.addLine("Drive By Distance");
//        telemetry.update();
//        driveWithPixy(bColor, 0.3, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//        DriveByDistance(20, -0.4, range, modeTurn);
////        getToSideCol(3, modeTurn, 0.4, 0.8);
//        sleep(nSleep);
//        scoreGlip(modeTurn);

    }


    public void getMoreGlyph1(double power, int color, int angle) {//0= red,1= blue
        int resume = 1;
        servoGlipSides.setPosition(1);
        intakeDC[0].setPower(-0.8);
        intakeDC[1].setPower(0.8);
        double time = getRuntime();
        Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                (int) (sensorColorBack.green() * SCALE_FACTOR),
                (int) (sensorColorBack.blue() * SCALE_FACTOR),
                hsvValues);
        while (opModeIsActive() && sensorColorDistanse.getDistance(DistanceUnit.CM) > 10 && time - getRuntime() < 0.7) {
            setMotorPower(new double[][]{{power, power}, {power, power}});

            // drive to
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(nSleep);
        setMotorPower(new double[][]{{-power, -power}, {-power, -power}});
        sleep(250);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(nSleep);

        while (opModeIsActive() && sensorColorDistanse.getDistance(DistanceUnit.CM) > 10 && time - getRuntime() < 0.6) {
            setMotorPower(new double[][]{{power, power}, {power, power}});

        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(nSleep);
        intakeDC[0].setPower(0.8);
        intakeDC[1].setPower(-0.8);
        driveByColor(color, angle, -0.3);
        intakeDC[0].setPower(0);
        intakeDC[1].setPower(0);

    }

    public double Pixy(int color) {
        pixyCam.engage();

        pixyData1 = pixyCam.read(0x51, 5);
        pixyData2 = pixyCam.read(0x52, 5);


        x1 = pixyData1[1];
        y1 = pixyData1[2];
        width1 = pixyData1[3];
        height1 = pixyData1[4];
        numObjects1 = pixyData1[0];

        x2 = pixyData2[1];
        y2 = pixyData2[2];
        width2 = pixyData2[3];
        height2 = pixyData2[4];
        numObjects2 = pixyData2[0];

        if (x2 < 90) {
            if (x2 < 135) {
                telemetry.addData("pixy in the right", 0xff & pixyData2[1]);
                telemetry.update();
            }
        }

        if (x2 > 170) {
            if (x2 > 135) {
                telemetry.addData("pixy in the left", 0xff & pixyData2[1]);
                telemetry.update();
            }
        }

        if (x2 == 0) {
            telemetry.addData("pixy dont see", 0xff & pixyData2[1]);
            telemetry.update();
        }

//            telemetry.addData("1.0", 0xff&pixyData1[0]);
//            telemetry.addData("1.1", 0xff&pixyData1[1]);
//            telemetry.addData("1.2", 0xff&pixyData1[2]);
//            telemetry.addData("1.3", 0xff&pixyData1[3]);
//            telemetry.addData("1.4", 0xff&pixyData1[4]);
//            telemetry.addData("1.Length", pixyData1.length);
//
//            telemetry.addData("2.0", 0xff&pixyData2[0]);
        telemetry.addData("2.1", 0xff & pixyData2[1]);
//            telemetry.addData("2.2", 0xff&pixyData2[2]);
//            telemetry.addData("2.3", 0xff&pixyData2[3]);
//            telemetry.addData("2.4", 0xff&pixyData2[4]);
//            telemetry.addData("2.Length", pixyData2.length);

        telemetry.update();

        return pixyData2[1];
    }

    public void getToSideCol(int Vu, double heading, double power, double timeStarf) {
        double pidErr[] = {0, 0};
        if (range.getDistance(DistanceUnit.CM) == 0) {
            telemetry.addLine("break");
            telemetry.update();
            sleep(5000);
        }
        pidErr = GyroPID(0, pidErr[1]);
        telemetry.addData("START DIST", range.getDistance(DistanceUnit.CM));
        telemetry.update();
        DriveByDistance(20, -0.3, range, heading);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(nSleep);
        telemetry.addLine("finished");
        telemetry.update();
        if (Vu == 1 || Vu == 2) {
            telemetry.addLine("in IF");
            telemetry.update();
            double time = getRuntime();
            while (opModeIsActive() && getRuntime() < time + timeStarf) {
                setMotorPower(new double[][]{{-0.5, 0.5}, {0.5, -0.5}});
                telemetry.addData("runTime", getRuntime());
                telemetry.addLine("Vu1");
                telemetry.update();
            }
        }
        if (Vu == 3) {
            double time = getRuntime();
            while (opModeIsActive() && (getRuntime() < (time + timeStarf))) {
                setMotorPower(new double[][]{{0.5, -0.5}, {-0.5, 0.5}});
                telemetry.addLine("Vu3");
                telemetry.update();
            }
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }

    public void getDownWithColorAndGyro(double power, int color) {
        double startRoll = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        double pidErr[] = {0, 0};
        setMotorPower(new double[][]{{0, 0}, {power - pidErr[0], power + pidErr[0]}});
        if (color == 0) {


            while (imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle < (startRoll + 4) && opModeIsActive() && hsvValues[0] < 76)
                ;
            Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                    (int) (sensorColorBack.green() * SCALE_FACTOR),
                    (int) (sensorColorBack.blue() * SCALE_FACTOR),
                    hsvValues);
            while (imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle > (startRoll + 4) && opModeIsActive() && hsvValues[0] < 76) {
                telemetry.addData("roll", imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
                telemetry.update();
                pidErr = GyroPID(0, pidErr[1]);
                setMotorPower(new double[][]{{0, 0}, {power + pidErr[0], power - pidErr[0]}});
                Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                        (int) (sensorColorBack.green() * SCALE_FACTOR),
                        (int) (sensorColorBack.blue() * SCALE_FACTOR),
                        hsvValues);
            }
        }
        if (color == 1) {


            while (imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle < (startRoll + 4) && opModeIsActive() && hsvValues[0] > 105)
                ;
            Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                    (int) (sensorColorBack.green() * SCALE_FACTOR),
                    (int) (sensorColorBack.blue() * SCALE_FACTOR),
                    hsvValues);
            while (imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle > (startRoll + 4) && opModeIsActive() && hsvValues[0] > 105) {
                telemetry.addData("roll", imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
                telemetry.update();
                pidErr = GyroPID(0, pidErr[1]);
                setMotorPower(new double[][]{{0, 0}, {power + pidErr[0], power - pidErr[0]}});
                Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                        (int) (sensorColorBack.green() * SCALE_FACTOR),
                        (int) (sensorColorBack.blue() * SCALE_FACTOR),
                        hsvValues);
            }
        }
    }


    public void getDownWithColor(double power, int color) {
        double pidErr[] = {0, 0};

        if (color == 0) {
            Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                    (int) (sensorColorBack.green() * SCALE_FACTOR),
                    (int) (sensorColorBack.blue() * SCALE_FACTOR),
                    hsvValues);
            if (hsvValues[0] < 76) {
                while (opModeIsActive() && hsvValues[0] < 76) {

//                    pidErr = GyroPID(0, pidErr[1]);
                    setMotorPower(new double[][]{{0, 0}, {power - pidErr[0], power + pidErr[0]}});
                    telemetry.addLine("InDriveOutofRed");
                    telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                    telemetry.addData("hsvValues[0]", hsvValues[0]);
                    telemetry.update();
                    Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                            (int) (sensorColorBack.green() * SCALE_FACTOR),
                            (int) (sensorColorBack.blue() * SCALE_FACTOR),
                            hsvValues);
                }
            }
        }
        if (color == 1) {
            Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                    (int) (sensorColorBack.green() * SCALE_FACTOR),
                    (int) (sensorColorBack.blue() * SCALE_FACTOR),
                    hsvValues);
            if (hsvValues[0] > 105) {
                while (opModeIsActive() && hsvValues[0] > 105) {

                    pidErr = GyroPID(0, pidErr[1]);
                    setMotorPower(new double[][]{{0, 0}, {power - pidErr[0], power + pidErr[0]}});
                    telemetry.addLine("InDriveOutofRed");
                    telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                    telemetry.addData("hsvValues[0]", hsvValues[0]);
                    telemetry.update();
                    Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                            (int) (sensorColorBack.green() * SCALE_FACTOR),
                            (int) (sensorColorBack.blue() * SCALE_FACTOR),
                            hsvValues);
                }
            }
        }
    }

    public void driveWithPixy(int color, double pow, double heading) {
        double pixy = pixyf(color);
        double time0 = getRuntime();

        while (opModeIsActive() && pixy == 0 && getRuntime() - time0 < 1 && range.getDistance(DistanceUnit.CM) >= 20) {
            setMotorPower(new double[][]{{pow, pow}, {pow, pow}});
            pixy = Pixy(color);
        }
        if (pixy < 100)
            NewdriveRobotEncoder(10, 90, 0.2, heading);
        else if (pixy > 200)
            NewdriveRobotEncoder(10, -90, 0.2, heading);

        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }

    public double pixyf(int color) {
        pixyCam.engage();

        pixyData1 = pixyCam.read(0x51, 5);
        pixyData2 = pixyCam.read(0x52, 5);


        x1 = pixyData1[1];
        y1 = pixyData1[2];
        width1 = pixyData1[3];
        height1 = pixyData1[4];
        numObjects1 = pixyData1[0];

        x2 = pixyData2[1];
        y2 = pixyData2[2];
        width2 = pixyData2[3];
        height2 = pixyData2[4];
        numObjects2 = pixyData2[0];

        if (x2 < 90) {
            if (x2 < 135) {
                telemetry.addData("pixy in the right", 0xff & pixyData2[1]);
                telemetry.addData("pixy in the left", 0xff & pixyData2[1]);
                telemetry.update();
            }
        }

        if (x2 > 170) {
            if (x2 > 135) {
                telemetry.addData("pixy in the right", 0xff & pixyData2[1]);
                telemetry.addData("pixy in the left", 0xff & pixyData2[1]);
                telemetry.update();
            }
        }

        if (x2 == 0) {
            telemetry.addData("pixy dont see", 0xff & pixyData2[1]);
            telemetry.update();
        }

        telemetry.addData("pixy in the right", 0xff & pixyData2[1]);
        telemetry.addData("pixy in the left", 0xff & pixyData2[1]);
        telemetry.update();
        telemetry.addData("2.1", 0xff & pixyData2[1]);
        telemetry.update();
        if (color == 0)
            return (pixyData1[1]);
        else if (color == 1)
            return pixyData2[1];
        else
            return pixyData2[1];
    }

    public void driveByColor(int color, double heading, double power)//0=red, blue=1
    {

        Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                (int) (sensorColorBack.green() * SCALE_FACTOR),
                (int) (sensorColorBack.blue() * SCALE_FACTOR),
                hsvValues);
        double pidErr[] = {0, 0};
        telemetry.addData("hsvValues[0]", hsvValues[0]);
        telemetry.update();
        if (color == 0 && opModeIsActive()) {
            double time = getRuntime();
            while (opModeIsActive() && hsvValues[0] > 76 && (time + 1.5 > getRuntime())) {
                Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                        (int) (sensorColorBack.green() * SCALE_FACTOR),
                        (int) (sensorColorBack.blue() * SCALE_FACTOR),
                        hsvValues);
                pidErr = GyroPID(heading, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
                telemetry.addLine("InDriveRed");
                telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("hsvValues[0]", hsvValues[0]);
                telemetry.update();

            }
//            double time2 = getRuntime();
//            while (opModeIsActive() && hsvValues[0] > 76 && (time2 + 1.5 > getRuntime())) {
//                Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
//                        (int) (sensorColorBack.green() * SCALE_FACTOR),
//                        (int) (sensorColorBack.blue() * SCALE_FACTOR),
//                        hsvValues);
//                pidErr = GyroPID(heading, pidErr[1]);
//                setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
//                telemetry.addLine("InDriveRed");
//                telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
//                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//                telemetry.addData("hsvValues[0]", hsvValues[0]);
//                telemetry.update();
//
//            }
            if (color == 1 && opModeIsActive()) {
                double time1 = getRuntime();

                while (opModeIsActive() && hsvValues[0] < 108 && (time1 + 1.5 > getRuntime())) {
                    Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                            (int) (sensorColorBack.green() * SCALE_FACTOR),
                            (int) (sensorColorBack.blue() * SCALE_FACTOR),
                            hsvValues);
                    pidErr = GyroPID(heading, pidErr[1]);
                    setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
                    telemetry.addLine("InDriveBlue");
                    telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                    telemetry.addData("hsvValues[0]", hsvValues[0]);
                    telemetry.update();

                }
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            sleep(nSleep);
        }
    }

    public void DriveByDistance(double distance, double power, ModernRoboticsI2cRangeSensor rangeSensor, double heading) {
        double pidErr[] = {0, 0};

//        if (rangeSensor.getDistance((DistanceUnit.CM)) == 0)
//        {
//            telemetry.addLine("rengeSensor=0");
//            telemetry.update();
//            sleep(5);
        if (distance > rangeSensor.getDistance(DistanceUnit.CM)) {
            telemetry.addData("HERE", rangeSensor.getDistance((DistanceUnit.CM)));
            telemetry.update();
            double time = getRuntime();
            while (opModeIsActive() && ((rangeSensor.getDistance(DistanceUnit.CM) < distance) && getRuntime() < (time + distance / 10))) {
                pidErr = GyroPID(heading, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
                telemetry.addData("distance1", rangeSensor.getDistance((DistanceUnit.CM)));
                telemetry.update();
                sleep(200);
            }
        } else if ((distance) < rangeSensor.getDistance(DistanceUnit.CM)) {
            double time = getRuntime();

            while (opModeIsActive() && (distance < rangeSensor.getDistance(DistanceUnit.CM)) && getRuntime() < (time + distance / 10)) {
                pidErr = GyroPID(heading, pidErr[1]);
                setMotorPower(new double[][]{{-power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], -power + pidErr[0]}});
                telemetry.addData("distance4", rangeSensor.getDistance((DistanceUnit.CM)));
                telemetry.update();
            }
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(nSleep);
    }

    public void getDownBalance2(double power) {
        double startRoll = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        double pidErr[] = {0, 0};
        setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
        //sleep(nSleep);
        while (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle < (startRoll + 3) && opModeIsActive())
            ;

        while (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle > (startRoll + 2) && opModeIsActive()) {
            telemetry.addData("roll", imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
            telemetry.update();
            pidErr = GyroPID(0, pidErr[1]);
            setMotorPower(new double[][]{{power + pidErr[0], power - pidErr[0]}, {power + pidErr[0], power - pidErr[0]}});
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }

    public void getDownBalance(double power) {
        double startRoll = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        double pidErr[] = {0, 0};
        setMotorPower(new double[][]{{0, 0}, {power - pidErr[0], power + pidErr[0]}});
        while (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle < (startRoll + 4) && opModeIsActive())
            ;

        while (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle > (startRoll + 4) && opModeIsActive()) {
            telemetry.addData("roll", imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
            telemetry.update();
//            pidErr = GyroPID(0, pidErr[1]);
            setMotorPower(new double[][]{{0, 0}, {power + pidErr[0], power - pidErr[0]}});
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }

    public void getMoreGlyph(double power, int color, int angle) {//0= red,1= blue
        servoGlipSides.setPosition(1);
        intakeDC[0].setPower(-0.8);
        intakeDC[1].setPower(0.8);
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                (int) (sensorColorBack.green() * SCALE_FACTOR),
                (int) (sensorColorBack.blue() * SCALE_FACTOR),
                hsvValues);
        // while (opModeIsActive() && getRuntime() < 10 && (hsvValues[0] < 105 || hsvValues[0] > 76)) {
        //Turn(angle - imu.getAngularOrientation(AxesReference.INTRINSIC,
        //   AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        //    while (opModeIsActive()&&getRuntime()<100 )/*(sensorGlip1.blue() <= 100 || sensorGlip2.blue() <= 100))*/
        Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                (int) (sensorColorBack.green() * SCALE_FACTOR),
                (int) (sensorColorBack.blue() * SCALE_FACTOR),
                hsvValues);
        double time = getRuntime();
        //while (opModeIsActive()&&hsvValues[0]<105&&hsvValues[0]>76&&time-getRuntime()<4.5){
        Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                (int) (sensorColorBack.green() * SCALE_FACTOR),
                (int) (sensorColorBack.blue() * SCALE_FACTOR),
                hsvValues);

        setMotorPower(new double[][]{{power, power}, {power, power}});
        sleep(750);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(nSleep);
        setMotorPower(new double[][]{{-power, -power}, {-power, -power}});
        sleep(250);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(nSleep);
        setMotorPower(new double[][]{{power, power}, {power, power}});
        sleep(650);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(nSleep);

        driveByColor(color, angle, 0.27);
//            telemetry.addLine("Turn -45");
//            telemetry.update();
//            Turn(-20);
//            setMotorPower(new double[][]{{power, power}, {power, power}});
//            sleep(1500);
//            setMotorPower(new double[][]{{0, 0}, {0, 0}});
//            sleep(nSleep);
//            driveByColor(color, angle, 0.3);
//            telemetry.addLine("Turn 90");
//            telemetry.update();
//            Turn(40);
//            setMotorPower(new double[][]{{power, power}, {power, power}});
//            sleep(1500);
//            setMotorPower(new double[][]{{0, 0}, {0, 0}});
//            sleep(nSleep);
//            driveByColor(color, angle, 0.3);

// It is important to remember to use encoders to mark a location in Crypto Box and where we are after collecting the glyphs


    }


    public void scoreGlip(double heading) {

        double pos;
        double scoreP = 0.8;
        double sideP = 0;

        //servoGlipBring.setPosition(0.8);
        servoGlipSides.setPosition(0.83);
        sideP = 0.83;
        while (opModeIsActive() && pot.getVoltage() <= 1.1) {
            scoreP += 0.005;
            servoGlipBring.setPosition(scoreP);
//            telemetry.addData("bring", scoreP);
//            telemetry.update();
        }
        //sleep(500);
        while (opModeIsActive() && sideP > 0) {
            sideP -= 0.05;
            servoGlipSides.setPosition(sideP);
//            telemetry.addData("sidess", sideP);
//            telemetry.update();
        }
        sleep(nSleep);
//        double nowDist = range.getDistance(DistanceUnit.CM);
        //   DriveByDistance(25, -0.4, range, imu.getAngularOrientation(AxesReference.INTRINSIC,
        //           AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        NewdriveRobotEncoder(7, 0, -0.4, imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        sleep(nSleep);
//        while (opModeIsActive() && sideP > 0) {
//            sideP -= 0.005;
//            servoGlipSides.setPosition(sideP);
////            telemetry.addData("sidesBack", sideP);
////            telemetry.update();
//        }
        telemetry.addLine("in Push");

        NewdriveRobotEncoder(15, 0, 0.5, imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addLine("in Go Back");
        sleep(nSleep);

        NewdriveRobotEncoder(15, 0, -0.4, imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        sleep(nSleep);
        servoGlipSides.setPosition(0.9);

//        while (opModeIsActive() && scoreP < 0.8) {
//            scoreP += 0.005;
//            servoGlipBring.setPosition(scoreP);
//            telemetry.addData("Servo Bring Back", scoreP);
//            telemetry.update();
//        }

//        sleep(nSleep);
        while (opModeIsActive() && pot.getVoltage() > 0.31) {
            scoreP -= 0.005;
            servoGlipBring.setPosition(scoreP);
            telemetry.addData("bringBack", scoreP);
            telemetry.update();
        }
        servoGlipSides.setPosition(0);


    }

    public void NewdriveRobotEncoder(double goalDist, double direction, double k, double heading) {
        if (opModeIsActive()) {
//dc motor [0][0] not working
            dcmotor[1][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[1][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcmotor[0][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[0][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcmotor[1][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[1][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double pidErr[] = {0, 0};

            pidErr = GyroPID(heading, pidErr[1]);
            double deg2rad = direction / 180 * Math.PI;
            double currDist = 0;
            double err = goalDist;
            double time0 = getRuntime();

            while (opModeIsActive() && (err) > 2 && (getRuntime() - time0) < goalDist / 10) {
                if (err > 0) {
                    dcmotor[0][0].setPower(k * (Math.cos(deg2rad) + Math.sin(deg2rad) - pidErr[0]));
                    dcmotor[1][0].setPower(k * (Math.cos(deg2rad) - Math.sin(deg2rad) + pidErr[0]));
                    dcmotor[0][1].setPower(k * (Math.cos(deg2rad) - Math.sin(deg2rad)) - pidErr[0]);
                    dcmotor[1][1].setPower(k * (Math.cos(deg2rad) + Math.sin(deg2rad)) + pidErr[0]);
                }

                currDist = Math.abs((
                                (dcmotor[1][0].getCurrentPosition() * cmRound / tixRound) +
                                        (dcmotor[0][1].getCurrentPosition() * cmRound / tixRound) +
                                        (dcmotor[1][1].getCurrentPosition() * cmRound / tixRound)
                        ) / 3
                );
                err = goalDist - currDist;
                telemetry.addData("currDist", currDist);
                telemetry.addData("dcmotor[1][0]", (dcmotor[1][0].getCurrentPosition()));
                telemetry.addData("dcmotor[0][1]", (dcmotor[0][1].getCurrentPosition()));
                telemetry.addData("dcmotor[1][1]", (dcmotor[1][1].getCurrentPosition()));
                telemetry.addData("current distance: ", currDist);
                telemetry.addData("current error: ", err);
                telemetry.update();
            }
        }
        setMotorPower(new double[][]{{0.0, 0.0}, {0.0, 0.0}});

    }

    public void newTurn(double angle) {
        double newAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + angle;
        double power = 0;
        double pidErr[] = {0, angle};
        setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});

        while (Math.abs(pidErr[1]) > 2 && opModeIsActive()) {
            telemetry.addData("roll", imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
            pidErr = GyroPID(newAngle, pidErr[1]);
            setMotorPower(new double[][]{{power + pidErr[0], power - pidErr[0]}, {power + pidErr[0], power - pidErr[0]}});
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }

    public double[] GyroPID(double heading, double lasterror) {
        double kp = 0.03, kd = 0.01, ki = 0, nexterror = 0;
        double err = heading - imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        lasterror = err - lasterror;
        double pd = nexterror * ki + lasterror * kd + err * kp;
        return (new double[]{-pd, lasterror});
    }

    public void Turn(double angle) {
        telemetry.addLine("IN TURN:");
        telemetry.update();
        double angle0 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double target = angle0 + angle;
        boolean flag1 = false;
        if (angle < 0) {
            if (target < -180)
                flag1 = true;
            setMotorPower(new double[][]{{-0.7, 0.7}, {-0.7, 0.7}});
            double angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if (flag1)
                angle1 -= 360;
            while (angle1 > target + 15 && opModeIsActive()) {
                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (flag1)
                    angle1 -= 360;
            }

            setMotorPower(new double[][]{{-0.27, 0.27}, {-0.27, 0.27}});
            double t0 = getRuntime();
            while ((angle1 - target > 2) && opModeIsActive() && getRuntime() - t0 < 1.2) {
                double pow = 0.2 * (-angle1 + target) / 15 + 0.3;
                setMotorPower(new double[][]{{-pow, pow}, {-pow, pow}});
                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (flag1)
                    angle1 -= 360;
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
        }
        if (angle > 0) {
            if (target > 180)
                flag1 = true;
            setMotorPower(new double[][]{{+0.7, -0.7}, {+0.7, -0.7}});
            double angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if (flag1)
                angle1 += 360;
            while ((angle1 < target - 15) && opModeIsActive()) {

                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (flag1)
                    angle1 += 360;
            }
            double t0 = getRuntime();
            setMotorPower(new double[][]{{+0.27, -0.27}, {+0.27, -0.27}});
            while (angle1 - target < -2 && opModeIsActive() && getRuntime() - t0 < 1.2) {
                double pow = 0.2 * (-angle1 + target) / 15 + 0.3;
                setMotorPower(new double[][]{{pow, -pow}, {pow, -pow}});
                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (flag1)
                    angle1 += 360;
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
        }
    }


    public void setMotorPower(double[][] power) {
        for (int row = 0; opModeIsActive() && row < 2; row++)
            for (int col = 0; opModeIsActive() && col < 2; col++)
                dcmotor[row][col].setPower(power[row][col]);
    }


//
//
//            if (angle > 0) {
//            angle -= 15;
//            setMotorPower(new double[][]{{0.3, -0.3}, {0.3, -0.3}});
//            double angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
//                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//            double robotError = (angle);
//            while (robotError > 180)  robotError -= 360;
//            while (robotError <= -180) robotError += 360;
//
//            while (opModeIsActive() && (robotError) > 15) {
//                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
//                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//                robotError = (angle1 - angle0);
//                while (robotError > 180)  robotError -= 360;
//                while (robotError <= -180) robotError += 360;
//                telemetry.addData("angle1: ", angle1);
//                telemetry.addData("angle0: ", angle0);
//                telemetry.update();
//                if (angle1 - angle0 < 0) {
//                    angle1 += 360;
//                    telemetry.addData("DONE: ", angle1);
//                    telemetry.update();
//                }
//            }
//            setMotorPower(new double[][]{{0.24, -0.24}, {0.24, -0.24}});
//            while (opModeIsActive() && (robotError) > 2) {
//                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
//                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//                robotError = (angle1 - angle0);
//                while (robotError > 180)  robotError -= 360;
//                while (robotError <= -180) robotError += 360;
//                if (angle1 - angle0 < 0) {
//                    angle1 += 360;
//                    telemetry.addData("DONE: ", angle1);
//                    telemetry.update();
//                }
//                telemetry.addData("angle1", angle1);
//                telemetry.addData("angle0", angle0);
//                telemetry.update();
//
//
//            }
//            setMotorPower(new double[][]{{0, 0}, {0, 0}});
//            telemetry.addData("angleAfter", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//        } else if (angle < 0) {
//            angle += 15;
//            double angle2 = imu.getAngularOrientation(AxesReference.INTRINSIC,
//                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//            double robotError = (angle2 - angle0);
//            while (robotError > 180)  robotError -= 360;
//            while (robotError <= -180) robotError += 360;
//            setMotorPower(new double[][]{{-0.3, 0.3}, {-0.3, 0.3}});
//            while (opModeIsActive() && angle2 - angle0 > 15) {
//                robotError = (angle2 - angle0);
//                while (robotError > 180)  robotError -= 360;
//                while (robotError <= -180) robotError += 360;
//                angle2 = imu.getAngularOrientation(AxesReference.INTRINSIC,
//                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//                telemetry.addData("angle2: ", angle2);
//                telemetry.addData("angle0: ", angle0);
//                telemetry.update();
//                if (angle2 > 0)
//                    angle2 -= 360;
//            }
//
//            setMotorPower(new double[][]{{-0.24, 0.24}, {-0.24, 0.24}});
//            while (opModeIsActive() && Math.abs(robotError) > 2) {
//                angle2 = imu.getAngularOrientation(AxesReference.INTRINSIC,
//                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//                robotError = (angle2 - angle0);
//                while (robotError > 180)  robotError -= 360;
//                while (robotError <= -180) robotError += 360;
//                if (angle2 > 0)
//                    angle2 -= 360;
//                telemetry.addData("angle2 second", angle2);
//                telemetry.addData("angle0 second", angle0);
//                telemetry.update();
//
//            }ne
//
//        }
//        setMotorPower(new double[][]{{0, 0}, {0, 0}});
//        telemetry.addData("angleAfter", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


    public void colDrive(ModernRoboticsI2cRangeSensor range, int direction, double power, int colVu) {//1= red , 2=blue
        // get reference rangea
        double range0 = range.getDistance(DistanceUnit.CM);
        double pidErr[] = {0, 0};
        int i0 = 1;
        // start moving
        if (direction == 0) {
            power = power;
        }
        if (direction == 1) {
            power = -power;
        }
        telemetry.addData("range0: ", range0);
        telemetry.update();

//            telemetry.addLine("going forward");
        sleep(nSleep);

        //setMotorPower(new double[][]{{-power, -power}, {-power, -power}});
        // search for the first col
        DriveByDistance(20, -Math.abs(power) * 0.5, range, modeTurn);
        telemetry.addData("range1: ", range0);
        telemetry.update();

        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(nSleep);
        range0 = range.getDistance(DistanceUnit.CM);
        pidErr[1] = 0;
        setMotorPower(new double[][]{{power, -power}, {-power, power}});
        // search for the first col
        double t0 = getRuntime();
        while (opModeIsActive() && range0 - range.getDistance(DistanceUnit.CM) < 4 && getRuntime() - t0 < 1.2) {
            pidErr = GyroPID(modeTurn, pidErr[1]);
            setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
            telemetry.addData("range first: ", range.getDistance(DistanceUnit.CM) - range0);
            telemetry.update();
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(nSleep);

        // search for the correct col
        double time = getRuntime();
        for (int iCol = i0; iCol < colVu && opModeIsActive() && getRuntime() < time + 6; iCol++) {
            telemetry.addData("goto", colVu);
            telemetry.addData("icol", iCol);
            telemetry.update();
            sleep(nSleep);

            range0 = range.getDistance(DistanceUnit.CM);

            while (opModeIsActive() && (range0 - range.getDistance(DistanceUnit.CM)) > -5) {
                pidErr = GyroPID(modeTurn, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
                telemetry.addData("icol2", iCol);
                telemetry.addData("rangecol: ", range0 - range.getDistance(DistanceUnit.CM));
                telemetry.update();
                sleep(nSleep);
            }
            range0 = range.getDistance(DistanceUnit.CM);
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            sleep(nSleep);
            while (opModeIsActive() && range0 - range.getDistance(DistanceUnit.CM) < 6) {
                pidErr = GyroPID(modeTurn, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
                telemetry.addData("icol", iCol);
                telemetry.addData("rangemid: ", range0 - range.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            sleep(nSleep);

        }
        telemetry.addLine("finito");
        telemetry.update();
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        //drive forward
    }

    public int GetColoumn() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = "AVMHpAz/////AAAAGcjV1wtssUJ8qDsdO8BrsiRE9AjOOxR2alp+D4tJNIKvrBbofJ+N4UD+bTze24nU/Dc9BqPcJ4f+0ZVQfGNxy40x+4U+fUB6h7a6RotwgBbPn0TZmLtqPzRzGVGx+t1buWk36b34SV7otKsNLgvf1lnUKlffmWjrIr8vbfQEsZQf4SpIzPL6i9f4Bvki4DnHf+9OX4kZ6kS1PES5WWsx6N7WIkriiYCYEa/jBFhSfG1dlOqiUDI4QZX07PEO7rlYxi+bOaokIrccDU29zD7NvYjRAUkVCvlJNZ+w20CPHIhqspfBKv5mMwiS5VXn2JzVhjwsPxaRWyTiN8bc5nZyb/MTGLIWf/E8T4axTE2fOAwT";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        int col = 2;
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        //   while (!gamepad1.a) {vu

        while (!opModeIsActive() && !isStarted()/*&&vuMark == RelicRecoveryVuMark.UNKNOWN*/) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            double time0 = getRuntime();
            //while (opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN && (getRuntime() - time0) < 1) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            // sleep(nSleep);
            telemetry.addData("VuMark", "%s visible", vuMark);

            telemetry.update();

            //}
            sleep(nSleep);
            telemetry.addLine("VuMark FOUND");
            telemetry.update();


            if (mode == 1 || mode == 2) {
                if (vuMark == RelicRecoveryVuMark.LEFT)
                    col = 3;
                if (vuMark == RelicRecoveryVuMark.CENTER)
                    col = 2;
                if (vuMark == RelicRecoveryVuMark.RIGHT)
                    col = 1;
            }
            if (mode == 3 || mode == 4) {
                if (vuMark == RelicRecoveryVuMark.LEFT)
                    col = 1;
                if (vuMark == RelicRecoveryVuMark.CENTER)
                    col = 2;
                if (vuMark == RelicRecoveryVuMark.RIGHT)
                    col = 3;
            }
        }
        sleep(nSleep);

        // relicTrackables.deactivate();
        return col;

    }


    public void knockBall(int color) {
//        double pos1 = 0.8;  //0.4 is the jwells
//        double pos2 = 0.5;  //0.35 to all sides. 0.5 is between
        // 0=red,1=blue
        telemetry.addLine("IN KnockBall");
        telemetry.update();

        if (opModeIsActive()) {
//            servoArm.setPosition(0.3);
//            sleep(nSleep);

            double pos = 0.6;

            while (opModeIsActive() && pos > 0) {
                pos -= 0.005;
                servoSensor.setPosition(pos);
//                sleep(50);
                telemetry.addData("IN SERVO DOWN", pos);
                telemetry.update();
            }
            sleep(1200);
            if (opModeIsActive() && ((sensorColor.blue() > sensorColor.red() && color == 1) ||
                    (sensorColor.blue() < sensorColor.red() && color == 0))) {
                telemetry.addData(">", "Color Checked", servoSensor.getPosition());
                telemetry.update();
                servoArm.setPosition(0);
                sleep(300);
            }
            else if (opModeIsActive() && ((sensorColor.red() > sensorColor.blue() && color == 1) ||
                    (sensorColor.red() < sensorColor.blue() && color == 0))) {
                telemetry.addData(">", "ColorNoRight", servoArm.getPosition());
                telemetry.update();
                servoArm.setPosition(1);
                sleep(300);
            }
//            } else {
//                servoArm.setPosition(0.9);
//                telemetry.addData(">", "in else", servoSensor.getPosition());
//                telemetry.update();
//            }

            servoArm.setPosition(0.6);
            pos = 0;
            while (pos < 0.7 && opModeIsActive()) {
                servoSensor.setPosition(pos);
//                sleep(nSleep);
                pos += 0.02;
//                sleep(50);
                telemetry.addData("servoSensor UP", pos);
                telemetry.update();
            }
//            servoArm.setPosition(0.5);
//
//            sleep(100);
            // servoSensor.setPosition(0.8);
            //  sleep(500);
            // servoArm.setPosition(1);

            //     servoArm.close();
            //     servoSensor.close();
            //  hardwareMap.servo.remove("servoSensor");
            //     hardwareMap.servo.remove("servoArm");

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
//
//    }

}