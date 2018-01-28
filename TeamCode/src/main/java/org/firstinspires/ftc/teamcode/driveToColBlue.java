package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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


@Autonomous(name = "driveToColBlue")

public class driveToColBlue extends LinearOpMode {


    OpenGLMatrix lastLocation = null;
    ElapsedTime runtime = new ElapsedTime();
    double distanceToGo = 0;
    VuforiaLocalizer vuforia;
    Servo servoGlipSides;

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
    int mode=3;
    RelicRecoveryVuMark v;
    DcMotor[] intakeDC = new DcMotor[2];


    @Override

    public void runOpMode() throws InterruptedException {


        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sRange");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        dcmotor[0][1] = hardwareMap.get(DcMotor.class, "right1");
        dcmotor[1][1] = hardwareMap.get(DcMotor.class, "right2");
        dcmotor[0][0] = hardwareMap.get(DcMotor.class, "left1");
        dcmotor[1][0] = hardwareMap.get(DcMotor.class, "left2");
        intakeDC[0] = hardwareMap.get(DcMotor.class, "g1");
        intakeDC[1] = hardwareMap.get(DcMotor.class, "g2");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColor");

        dcmotor[0][1].setDirection(DcMotorSimple.Direction.REVERSE);
        dcmotor[1][1].setDirection(DcMotorSimple.Direction.REVERSE);
        servoGlipSides = hardwareMap.get(Servo.class, "servoSides");
        //servoGlipHand = hardwareMap.get(Servo.class, "servoHand");
        servoGlipBring = hardwareMap.get(Servo.class, "servoBring");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoSensor = hardwareMap.get(Servo.class, "servoSensor");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        int bColor = 0;
        double distance = 0;
        telemetry.addData("IN SERVO init", servoSensor.getPosition());
        telemetry.update();

        telemetry.update();

        servoArm.setPosition(0.5);
        servoSensor.setPosition(0.8);
        switch (mode) {
            case 1: {//red 1
                modeTurn = 90;
                S = 0;
                D = 1;
                modeOri = 0;
                bColor = 0;
                break;
            }
            case 2: {//red 2
                modeTurn = 180;
                S = 0;
                D = 1;
                modeOri = 0;
                bColor = 0;
                break;
            }
            case 3: {//blue 1
                modeTurn = 90;
                S = 0;
                D = -1;
                modeOri = 180;
                bColor = 1;
                break;
            }
            case 4: {//blue 2
                modeTurn = 0;
                S = 15;
                D = -1;
                modeOri = 180;
                bColor = 1;

            }
        }
        double pos = 0;

//        telemetry.addData("Mode", mode);
//        telemetry.update();


        waitForStart();
        if (opModeIsActive()) {
            servoGlipSides.setPosition(0);
            telemetry.addLine("knockBall");
            telemetry.update();
            knockBall(bColor);
            telemetry.addLine("vufuria");
            telemetry.update();
            sleep(100);
            int colVu = GetColoumn();
            sleep(100);
            telemetry.addLine("getDownBalance");
            telemetry.update();
            sleep(100);
            getDownBalance(-0.5 * D, 200);
            sleep(500);
            telemetry.addLine("Turn");
            telemetry.update();
            double angle0 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            Turn(modeTurn - angle0);
            sleep(200);
            telemetry.addLine("colDrive");
            telemetry.update();
//         colDrive(range, bColor, -0.55*D, colVu);
//         sleep(200);
            colDrive(range, bColor, -0.65, colVu);
            sleep(200);
            telemetry.addLine("DriveByDistance");
            telemetry.update();
//         DriveByDistance(10,-0.4*D,range,imu.getAngularOrientation(AxesReference.INTRINSIC,
//                 AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            //DriveByDistance(10,-0.4,range,imu.getAngularOrientation(AxesReference.INTRINSIC,
            //  AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            sleep(200);
            telemetry.addLine("scoreGlip");
            telemetry.update();
            scoreGlip();
            sleep(200);
            // getMoreGlip(0,1,0.5);
        }
    }


    public void DriveByDistance(double distance, double power, ModernRoboticsI2cRangeSensor rangeSensor, double heading) {
        double pidErr[] = {0, 0};
        if (distance > rangeSensor.getDistance(DistanceUnit.CM)) {
            telemetry.addData("HERE", range.getDistance((DistanceUnit.CM)));
            telemetry.update();
            sleep(500);

            while (opModeIsActive() && (rangeSensor.getDistance(DistanceUnit.CM) < distance)) {
                pidErr = GyroPID(heading, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
                telemetry.addData("distance1", range.getDistance((DistanceUnit.CM)));
                telemetry.update();
            }
        } else if ((distance) < rangeSensor.getDistance(DistanceUnit.CM)) {
            while (distance < rangeSensor.getDistance(DistanceUnit.CM) && opModeIsActive()) {
                //pidErr = GyroPID(heading, pidErr[1]);
                setMotorPower(new double[][]{{-power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], -power + pidErr[0]}});
                telemetry.addData("distance4", range.getDistance((DistanceUnit.CM)));
                telemetry.update();
            }
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(200);
    }


    public void getDownBalance2(double power, int sleepTime) {
        double startRoll = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        double pidErr[] = {0, 0};
        setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
        //sleep(sleepTime);
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

    public void getDownBalance(double power, int sleepTime) {
        double startRoll = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        double pidErr[] = {0, 0};
        setMotorPower(new double[][]{{0, 0}, {power - pidErr[0], power + pidErr[0]}});
        //sleep(sleepTime);
        while (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle < (startRoll + 3) && opModeIsActive())
            ;

        while (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle > (startRoll + 2) && opModeIsActive()) {
            telemetry.addData("roll", imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
            telemetry.update();
//            pidErr = GyroPID(0, pidErr[1]);
            setMotorPower(new double[][]{{0, 0}, {power + pidErr[0], power - pidErr[0]}});
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }


    public void getMoreGlip(int howManyGlip, int nowGlip, double power) {
        //  servoGlipHand.setPosition(0);
        servoGlipSides.setPosition(0);
        intakeDC[0].setPower(0.7);
        intakeDC[1].setPower(0.7);
        DriveByDistance(200, power, range, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while (opModeIsActive() && nowGlip < howManyGlip) {
            if (sensorGlip.alpha() > 100)
                nowGlip++;

        }
        intakeDC[0].setPower(0);
        intakeDC[1].setPower(0);
        DriveByDistance(200, -power, range, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        scoreGlip();
    }


    public void scoreGlip() {
        double scoreP = 0;
        double sideP = 0;
        servoGlipBring.setPosition(0.25);
        servoGlipSides.setPosition(0);

        while (opModeIsActive() && scoreP < 0.5) {
            scoreP += 0.005;
            servoGlipBring.setPosition(scoreP);
            telemetry.addData("bring", scoreP);
            telemetry.update();
        }
        while (opModeIsActive() && sideP < 0.8) {
            sideP += 0.01;
            servoGlipSides.setPosition(sideP);
            telemetry.addData("sidess", sideP);
            telemetry.update();
        }

        double nowDist = range.getDistance(DistanceUnit.CM);
        DriveByDistance(nowDist + 5, -0.4, range, imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        sleep(200);
        while (opModeIsActive() && sideP > 0) {
            sideP -= 0.005;
            servoGlipSides.setPosition(sideP);
            telemetry.addData("sidesBack", sideP);
            telemetry.update();
        }
        //nowDist = range.getDistance(DistanceUnit.CM);
        DriveByDistance(nowDist, -0.4, range, imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        DriveByDistance(nowDist + 5, -0.4, range, imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        sleep(200);

        while (opModeIsActive() && scoreP > 0.25) {
            scoreP -= 0.005;
            servoGlipBring.setPosition(scoreP);
            telemetry.addData("Servo Bring Back", scoreP);
            telemetry.update();
        }


        sleep(500);
//        while (opModeIsActive() && scoreP > 0.25) {
//            scoreP -= 0.005;
//            servoGlipBring.setPosition(scoreP);
//            telemetry.addData("bringBack", scoreP);
//            telemetry.update();
//        }


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
        double kp = 0.05, kd = 0.01, ki = 0, nexterror = 0;
        double err = heading - imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        lasterror = err - lasterror;
        double pd = nexterror * ki + lasterror * kd + err * kp;
        return (new double[]{-pd, lasterror});
    }


    public void setMotorPower(double[][] power) {
        for (int row = 0; opModeIsActive() && row < 2; row++)
            for (int col = 0; opModeIsActive() && col < 2; col++)
                dcmotor[row][col].setPower(power[row][col]);
    }

    void Turn(double angle) {
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
            setMotorPower(new double[][]{{-0.4, 0.4}, {-0.4, 0.4}});
            while (angle1 > target + 1 && opModeIsActive()) {
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
            while (angle1 < target - 15 && opModeIsActive()) {
                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (flag1)
                    angle1 += 360;
            }
            setMotorPower(new double[][]{{+0.4, -0.4}, {+0.4, -0.4}});
            while (angle1 < target - 1 && opModeIsActive()) {
                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (flag1)
                    angle1 += 360;
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
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

    }


    public void colDrive(ModernRoboticsI2cRangeSensor range, int direction, double power, int colVu) {//1= red , 2=blue
        // get reference rangea
        double range0 = range.getDistance(DistanceUnit.CM);
        double pidErr[] = {0, 0};
        int i0 = 0;
        // start moving
        if (direction == 0) {
            power = power;
            i0 = 1;
        }
        if (direction == 1) {
            power = -power;
            i0 = 1;
        }
        telemetry.addData("range0: ", range0);
        telemetry.update();

//            telemetry.addLine("going forward");
        sleep(200);

        //setMotorPower(new double[][]{{-power, -power}, {-power, -power}});
        // search for the first col
        DriveByDistance(20, -Math.abs(power) * 0.5, range, modeTurn);
        telemetry.addData("range1: ", range0);
        telemetry.update();

        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(200);
        range0 = range.getDistance(DistanceUnit.CM);
        pidErr[1] = 0;
        setMotorPower(new double[][]{{power, -power}, {-power, power}});
        // search for the first col
        runtime.reset();
        while (opModeIsActive() && range0 - range.getDistance(DistanceUnit.CM) < 8 && runtime.seconds() < 2) {
                pidErr = GyroPID(modeTurn, pidErr[1]);
            setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
            telemetry.addData("range first: ", range.getDistance(DistanceUnit.CM) - range0);
            telemetry.update();
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        sleep(400);

        // search for the correct col
        runtime.reset();
        for (int iCol = i0; iCol < colVu && opModeIsActive() && runtime.seconds() < 4; iCol++) {
            telemetry.addData("goto", colVu);
            telemetry.addData("icol", iCol);
            telemetry.update();
            sleep(200);

            range0 = range.getDistance(DistanceUnit.CM);

            while (opModeIsActive() && (range0 - range.getDistance(DistanceUnit.CM)) > -5) {
                pidErr = GyroPID(modeTurn, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
                telemetry.addData("icol2", iCol);
                telemetry.addData("rangecol: ", range0 - range.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            range0 = range.getDistance(DistanceUnit.CM);
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            sleep(200);
            while (opModeIsActive() && range0 - range.getDistance(DistanceUnit.CM) < 6) {
                pidErr = GyroPID(modeTurn, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
                telemetry.addData("icol", iCol);
                telemetry.addData("rangemid: ", range0 - range.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            sleep(700);

        }
        telemetry.addLine("finito");
        telemetry.update();
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        //drive forward
    }

    int GetColoumn() {
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


        relicTrackables.activate();


        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        //   while (!gamepad1.a) {

        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        double time0 = getRuntime();
        while (opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN && (getRuntime() - time0) < 1) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            // sleep(200);
            telemetry.addData("VuMark", "%s visible", vuMark);

            telemetry.update();

        }
        sleep(200);

        telemetry.addLine("VuMark FOUND");
        //telemetry.addLine("VuMark FOUND",vuMark);
        telemetry.update();
        int col = 2;

        if (mode == 1 || mode == 2) {
            if (vuMark == RelicRecoveryVuMark.LEFT)
                col = 2;
            if (vuMark == RelicRecoveryVuMark.CENTER)
                col = 1;
            if (vuMark == RelicRecoveryVuMark.RIGHT)
                col = 0;
        }
        if (mode == 3 || mode == 4) {
            if (vuMark == RelicRecoveryVuMark.LEFT)
                col = 0;
            if (vuMark == RelicRecoveryVuMark.CENTER)
                col = 1;
            if (vuMark == RelicRecoveryVuMark.RIGHT)
                col = 2;
        }
        sleep(100);
      //  relicTrackables.deactivate();
        return col;

    }


    void knockBall(int color) {
//        double pos1 = 0.8;  //0.4 is the jwells
//        double pos2 = 0.5;  //0.35 to all sides. 0.5 is between
        // 0=red,1=blue
        telemetry.addLine("IN KnockBall");
        telemetry.update();

        if (opModeIsActive()) {
//            servoArm.setPosition(0.3);
//            sleep(500);


            double pos = 0.8;
            while (opModeIsActive() && pos > 0.4) {
                pos -= 0.0005;
                servoSensor.setPosition(pos);
                //sleep(1);
                telemetry.addData("IN SERVO DOWN", pos);
                telemetry.update();
            }
            sleep(200);
            if (opModeIsActive() && ((sensorColor.blue() > sensorColor.red() && color == 1) ||
                    (sensorColor.blue() < sensorColor.red() && color == 0))) {
                telemetry.addData(">", "Color Checked", servoSensor.getPosition());
                telemetry.update();
                servoArm.setPosition(1);
                sleep(200);
            } else if (opModeIsActive()) {
                telemetry.addData(">", "ColorNoRight", servoArm.getPosition());
                telemetry.update();
                servoArm.setPosition(0);
                sleep(200);


            }
            servoArm.setPosition(0.5);
            sleep(200);


            while (pos < 0.9 && opModeIsActive()) {
                servoSensor.setPosition(pos);
//                sleep(2);
                pos += 0.05;
                telemetry.addData("servoSensor UP", pos);
                telemetry.update();
            }
            // servoSensor.setPosition(0.8);
            sleep(400);
            servoArm.setPosition(1);

            //     servoArm.close();
            //     servoSensor.close();
            //  hardwareMap.servo.remove("servoSensor");
            //     hardwareMap.servo.remove("servoArm");


        }
    }
//
//    }

}