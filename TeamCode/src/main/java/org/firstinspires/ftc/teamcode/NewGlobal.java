package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

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

public class
NewGlobal extends LinearOpMode {
    HardwareMap aw = null;
    OpenGLMatrix lastLocation = null;


    ElapsedTime runtime = new ElapsedTime();
    double distanceToGo = 0;
    VuforiaLocalizer vuforia;
    Servo servoGlyphSides;
    DistanceSensor sensorColorDistanse;
    Servo[] servoGlyphBring = new Servo[2];

    DigitalChannel touchUp;
    DigitalChannel touchDown;

    public boolean col_ = false;
    DigitalChannel magnetDown;
    double blueColorRightSensor = 100;
    double redColorRightSensor = 42;
    double blueColorLeftSensor = 135;
    double redColorLeftSensor = 65;
    double blueColorMiddle = 135;
    double redColorMiddle = 65;
    Servo servoSensor;
    Servo servoArm;
    ColorSensor sensorColor;
    ColorSensor colorMiddle;
    ColorSensor sensorColorBack, sensorColorLeft;

    BNO055IMU imu, imu2;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    float hsvValues1[] = {0F, 0F, 0F};
    float hsvValuesMiddle[] = {0F, 0F, 0F};
    final float values1[] = hsvValues1;
    double cmRound = 29;
    double tixRound = 700;
    Orientation angles, angles2;
    ModernRoboticsI2cRangeSensor range;

    DcMotor[][] dcmotor = new DcMotor[5][4];
    int mode = 1;
    ModernRoboticsI2cRangeSensor rangeLeft;

    int modeTurn; // TURN PER MODE
    int S;// STRAFE PER MODE

    RelicRecoveryVuMark v;
    DcMotor[] intakeDC = new DcMotor[2];
    int nSleep = 100;
    double colDist = 23.5;
    double k;
    boolean colTime = false;
    double distFromWall = 26;
    final double SCALE_FACTOR = 255;
    boolean range0 = true;
    int bColor = 0;
    DcMotor dcRelic3;
    AnalogInput pot;
    Servo[] relicServo = new Servo[2];
    boolean glyphScore = true;

    public Boolean intakeSec = false;
    double voltageMin = 9.7;
    int colVu;
    DcMotor Door;

    private double integratedZAxis = 0;
    private double lastHeading = 0;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Boolean negative = false;

    @Override


    public void runOpMode() throws InterruptedException {

        // pixyCam = hardwareMap.get(I2cDeviceSynch.class, "pixy");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
        dcmotor[0][1] = hardwareMap.get(DcMotor.class, "right1");
        dcmotor[1][1] = hardwareMap.get(DcMotor.class, "right2");
        dcmotor[0][0] = hardwareMap.get(DcMotor.class, "left1");
        dcmotor[1][0] = hardwareMap.get(DcMotor.class, "left2");
        intakeDC[0] = hardwareMap.get(DcMotor.class, "g1");
        intakeDC[1] = hardwareMap.get(DcMotor.class, "g2");
        colorMiddle = hardwareMap.get(ColorSensor.class, "colorMiddle");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColor");
        sensorColorBack = hardwareMap.get(ColorSensor.class, "sensorColorRight");
        sensorColorLeft = hardwareMap.get(ColorSensor.class, "sensorColorLeft");
        relicServo[0] = hardwareMap.get(Servo.class, "relicServo1");
//        dcmotor[0][1].setDirection(DcMotorSimple.Direction.REVERSE);
//        dcmotor[1][1].setDirection(DcMotorSimple.Direction.REVERSE);
        servoGlyphSides = hardwareMap.get(Servo.class, "servoSides");
        touchUp = hardwareMap.get(DigitalChannel.class, "TouchUp");
        touchDown = hardwareMap.get(DigitalChannel.class, "TouchDown");
        rangeLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeLeft");
        dcRelic3 = hardwareMap.get(DcMotor.class, "relic3");
        magnetDown = hardwareMap.get(DigitalChannel.class, "magnetDown");
        //servoGlipHand = hardwareMap.get(Servo.class, "servoHand");
        servoGlyphBring[0] = hardwareMap.get(Servo.class, "servoBring");
        servoGlyphBring[1] = hardwareMap.get(Servo.class, "servoBring2");
        //Door = hardwareMap.get(DcMotor.class, "Door");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoSensor = hardwareMap.get(Servo.class, "servoSensor");
        sensorColorDistanse = (DistanceSensor) hardwareMap.get(DistanceSensor.class, "upperColorDistanse");
        pot = hardwareMap.get(AnalogInput.class, "pot");
        dcmotor[0][0].setDirection(DcMotorSimple.Direction.REVERSE);
        dcmotor[0][1].setDirection(DcMotorSimple.Direction.FORWARD);
        dcmotor[1][0].setDirection(DcMotorSimple.Direction.REVERSE);
        dcmotor[1][1].setDirection(DcMotorSimple.Direction.FORWARD);
        dcmotor[0][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmotor[0][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmotor[1][0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmotor[1][1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        resetZAxis();
        telemetry.addData("IN SERVO init", servoSensor.getPosition());
        telemetry.update();
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


        runtime.reset();
        relicServo[0].setPosition(0);
//        servoGlyphSides.setPosition(0);
        colVu = GetColoumn();
    }


    public void crossing(double power, double powerDC) {
        setMotorPower(new double[][]{{0, 0}, {0, 0}});//not moving while crossing
        intakeDC[0].setPower(-power);
        intakeDC[1].setPower(-power);
        sleep(100);
        setMotorPower(new double[][]{{-powerDC * 0.4, -powerDC * 0.4}, {-powerDC * 0.4, -powerDC * 0.4}});
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            intake(1);
            sleep(150);
        }


        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)) && readVoltage1() < voltageMin) {
            setMotorPower(new double[][]{{0, 0}, {0, 0}});//not moving while crossing
            intakeDC[0].setPower(-power);
            intakeDC[1].setPower(-power);
            sleep(100);
            setMotorPower(new double[][]{{-powerDC * 0.4, -powerDC * 0.4}, {-powerDC * 0.4, -powerDC * 0.4}});
            intake(1);
            sleep(200);
        }
//        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
//            intake(1);
//            sleep(100);
//        }

        reverseIntake(1);
        sleep(100);
        intake(1);
        sleep(175);
        stopIntake();
    }

    public void crosingWithDistance(double power, double distance, int number) {
        double time = getRuntime();
//        if (number == 1) {
        while (time + 0.15 > getRuntime() && opModeIsActive() && range.getDistance(DistanceUnit.CM) < distance) {
            intakeDC[0].setPower(-1);
            intakeDC[1].setPower(-1);
        }
        time = getRuntime();
        while (time + 0.2 > getRuntime() && opModeIsActive() && range.getDistance(DistanceUnit.CM) < distance) {
            intake(1);
        }
//        }

        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            time = getRuntime();
            while (time + 0.1 > getRuntime() && opModeIsActive() && range.getDistance(DistanceUnit.CM) < distance) {
                intakeDC[0].setPower(-1);
                intakeDC[1].setPower(-1);
            }
            time = getRuntime();
            while (time + 0.15 > getRuntime() && opModeIsActive() && range.getDistance(DistanceUnit.CM) < distance) {
                intake(1);
            }
        }
        time = getRuntime();
        while (time + 0.08 > getRuntime() && opModeIsActive() && range.getDistance(DistanceUnit.CM) < distance) {
            reverseIntake(1);
        }
        time = getRuntime();
        while (time + 0.175 > getRuntime() && opModeIsActive() && range.getDistance(DistanceUnit.CM) < distance) {
            intake(1);
        }

        stopIntake();
    }

    public void intake(double power) {
        intakeDC[0].setPower(-power);
        intakeDC[1].setPower(power);
    }

    public void intakeWithSensor(double power) {
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            intake(power);
        } else
            stopIntake();
    }

    public void stopIntake() {
        intakeDC[0].setPower(0);
        intakeDC[1].setPower(0);
    }

    public void reverseIntake(double power) {
        intakeDC[0].setPower(power); //Reverse intake
        intakeDC[1].setPower(-power);
    }

    public void DistanceAndDownDoor(double distance, double power) {

        double time = getRuntime();
//        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))){
//            servoGlyphSides.setPosition(0);
//            sleep(200);
//        }
        while (opModeIsActive() && range.getDistance(DistanceUnit.CM) + 2 < distance && magnetDown.getState() && !touchDown.getState() && getRuntime() - time < 1.5) {
            setMotorPower(new double[][]{{power, power}, {power, power}});
            servoGlyphBring[0].setPosition(1);
            servoGlyphBring[1].setPosition(0);


        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
        time = getRuntime();
        setMotorPower(new double[][]{{0, 0}, {0, 0}});

    }

    public void DistanceAndDownUp(double distance, double power) {

        double time = getRuntime();
//        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))){
//            servoGlyphSides.setPosition(0);
//            sleep(200);
//        }
        while (opModeIsActive() && range.getDistance(DistanceUnit.CM) + 2 < distance && !touchUp.getState() && Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)) && getRuntime() - time < 1) {
            setMotorPower(new double[][]{{power, power}, {power, power}});
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);

        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
        time = getRuntime();
        setMotorPower(new double[][]{{0, 0}, {0, 0}});

    }

    public void getAndScoreGlyph(int col, double turnPower, int numberGlyphPIT) {//1 for the First intake. 2 for the second intake.
//        driveByColorWith2Sensors(1, 25, 0.5, 180);

        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            if (numberGlyphPIT == 1)
                getGlyphFirst(-1);//intake Glyph first time
            if (numberGlyphPIT == 2)
                getGlyph(-1, 0);//intake glyph second time
        }
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            intake(1);//if 2 glyph dont up yet

        }

//        }
        driveByEncoder(33, 1, 0);
//        encoderAndCrosing(20, 0, 1, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//only in the secend time, go back
//        }
        stopIntake();

        if (numberGlyphPIT == 2) {
            intakeWithSensor(1);
            Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower + 0.2);
            Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower + 0.1);
            stopIntake();
        }
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            intake(0.8);//only if the 2 glyph dont up

        }

        setMotorPower(new double[][]{{1, 1}, {1, 1}});//move to the criptobox
        intakeDC[0].setPower(-1);
        intakeDC[1].setPower(-1);//crosing In move back
        sleep(100);
        intakeWithSensor(0.8);
        sleep(125);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});//stop
//        servoGlyphSides.setPosition(0);
        DistanceAndDownUp(40, 1);
        if (range.getDistance(DistanceUnit.CM) > 50) {
            DriveByDistance(50, 1, range, 0, 0);//drive with distance to the criptobox
        }

        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower);
//        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower - 0.15);
        DriveByDistance(distFromWall, 0.3, range, 0, 1);//drive with distance low

        if (!range0) {//if the range sensor not warking
            driveByEncoder(110, 0.4, 0);//go to the wall
            driveByEncoder(25, 1, 180);//go back
        }
        stopIntake();
//        intake(1);
//        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower);
        servoGlyphSides.setPosition(0);//close
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower - 0.15);


//        sleep(500);
        if (range.getDistance(DistanceUnit.CM) != 0) {//if range sensor working
//            driveByEncoder(5, 1, 90);
            colFindDistOrColor(range, 1, 0.55, 3, 1, 0);
        } else
//                driveByEncoder(colDist, 1, -90);
            colFindLiftColor(1, 0.55, 2.5, 1, 0);//if range sensor not warking, do colFind only with color
        if (hsvValuesMiddle[0] > blueColorMiddle) {
            driveByEncoder(10, 0.8, 90);
        } else {
            if (!colTime) {//if the colFind time NOT

                driveByEncoder(5, 1, 90);//go to the vofuria col
            }

//        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower);
//        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower - 0.15);

            driveByEncoder(colDist - 4, 0.8, -90);
        }
//        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower);
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower - 0.15);
        scoreGlyph(false);//put glyph
    }

    void resetZAxis() {

        lastHeading = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        integratedZAxis = 0;
    }

    public double getIntegratedZAxis() {

        double newHeading = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        double deltaHeading = newHeading - lastHeading;

        if (deltaHeading < -180)
            deltaHeading += 360;
        else if (deltaHeading >= 180)
            deltaHeading -= 360;

        integratedZAxis += deltaHeading;

        lastHeading = newHeading;

        return integratedZAxis;
    }

    public void ResetHue(ColorSensor color, float[] hsvArr) { //Reset the sensor color to read bt hue values.
        Color.RGBToHSV((int) (color.red() * SCALE_FACTOR),
                (int) (color.green() * SCALE_FACTOR),
                (int) (color.blue() * SCALE_FACTOR),
                hsvArr);
    }

    public void straightOnLine(int color, double power) {
        Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                (int) (sensorColorBack.green() * SCALE_FACTOR),
                (int) (sensorColorBack.blue() * SCALE_FACTOR),
                hsvValues);
        Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                (int) (sensorColorLeft.green() * SCALE_FACTOR),
                (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                hsvValues1);
        telemetry.addData("hsvValues[0]", hsvValues[0]);
        telemetry.update();
        if (color == 0) {

            double time = getRuntime();
            while (opModeIsActive() && hsvValues[0] > redColorRightSensor && hsvValues1[0] > redColorLeftSensor && (time + 1.5 > getRuntime())) {
                ResetHue(sensorColorBack, hsvValues);
                ResetHue(sensorColorLeft, hsvValues1);
                setMotorPower(new double[][]{{power, power}, {power, power}});
                telemetry.addLine("search the First Line");
                telemetry.addData("hsvValues[0]", hsvValues[0]);
                telemetry.update();
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});

            ResetHue(sensorColorBack, hsvValues);
            ResetHue(sensorColorLeft, hsvValues1);
            if (hsvValues[0] < redColorRightSensor) {
                while (hsvValues1[0] > redColorLeftSensor && opModeIsActive()) {
                    setMotorPower(new double[][]{{0.75 * power, 0}, {0.75 * power, 0}});
                    telemetry.addData("search the left Line", hsvValues1[0]);
                    telemetry.update();
                }
            }
            if (hsvValues1[0] < redColorLeftSensor) {
                while (hsvValues[0] > redColorRightSensor && opModeIsActive()) {
                    setMotorPower(new double[][]{{0, 0.75 * power}, {0, 0.75 * power}});
                    telemetry.addData("search the Right Line", hsvValues[0]);
                    telemetry.update();
                }
            }

        }
        if (color == 1) {

        }
    }

    public double readVoltage() {
        double result = Double.POSITIVE_INFINITY;
        int i = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            ++i;
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }

            telemetry.addData("voltage: ", result);
            telemetry.update();
        }
        //      telemetry.addData("voltage: ", result);

        return result;

    }

    public void getGlyph(double power, int color) { //Drives to the Glyph pit and gathers until 2 glyphs
        //0= red,1= blue

        servoGlyphSides.setPosition(1); //open glyphs holder
        intakeWithSensor(1);
        double time = getRuntime();

        telemetry.addData("sensor Range rev", sensorColorDistanse.getDistance(DistanceUnit.CM));
        telemetry.update();
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)))
            while (opModeIsActive() && getRuntime() - time < 0.4)//In loop until there is glyph pn the robot
            {

                setMotorPower(new double[][]{{power, power}, {power, power}});
                intakeWithSensor(1);
                telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
                telemetry.update();

            }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {

            intakeDC[0].setPower(-1);
            intakeDC[1].setPower(-1);
            sleep(125);
            intake(1);
            sleep(200);
        }
        stopIntake();

        intakeWithSensor(1);
        double angle = 12;
        if (color == 0)
            angle = -12;
        Turn(angle, 1);


        double minTime = 0.35;
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)))
            time = getRuntime();
        while (opModeIsActive() && getRuntime() - time < minTime)//In loop until there is glyph pn the robot
        {
            intakeWithSensor(1);
            setMotorPower(new double[][]{{power, power}, {power, power}});
            telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        intakeWithSensor(1);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            intakeDC[0].setPower(-1);
            intakeDC[1].setPower(-1);
            sleep(125);
            intake(1);
            sleep(200);
        }
        telemetry.addLine("Finished loop2");
        telemetry.update();


        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        stopIntake();

    }

    public void getGlyphFirst(double power) { //Drives to the Glyph pit and gathers until 2 glyphs
        //0= red,1= blue
        double volt = readVoltage();
        double pidErr[] = {0, 0};
        servoGlyphSides.setPosition(1); //open glyphs holder
        intake(1);
        double minTime = 0.4;

        double time = getRuntime();

        while (opModeIsActive() && getRuntime() - time < minTime)//In loop until there is glyph pn the robot
        {
            setMotorPower(new double[][]{{-power, -power}, {-power, -power}});

            intakeWithSensor(1);

            telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
        stopIntake();


        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        intakeDC[0].setPower(-1);
        intakeDC[1].setPower(-1);
        sleep(125);
        intake(1);
        sleep(200);


        telemetry.addLine("Finished loop1");
        telemetry.update();

        intakeWithSensor(1);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});

        setMotorPower(new double[][]{{power, power}, {power, power}});
        if (!Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            reverseIntake(1);
            sleep(100);
            intake(0.8);
            sleep(175);
        }

        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        stopIntake();

    }

    public void getGlyphNearNew(double power) { //Drives to the Glyph pit and gathers until 2 glyphs
        //0= red,1= blue


        servoGlyphSides.setPosition(1); //open glyphs holder
        intakeWithSensor(1);
        double time = getRuntime();

        telemetry.addData("sensor Range rev", sensorColorDistanse.getDistance(DistanceUnit.CM));
        telemetry.update();
        intakeWithSensor(1);
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)))
            Turn(-20, 1);
        setMotorPower(new double[][]{{power, power}, {power, power}});
        double minTime = 0.4;
        time = getRuntime();
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            while (opModeIsActive() && getRuntime() - time < minTime)//In loop until there is glyph pn the robot
            {
                setMotorPower(new double[][]{{power, power}, {power, power}});
                intakeWithSensor(1);
                telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
                telemetry.update();

            }
            stopIntake();
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            minTime = 0.3;
            setMotorPower(new double[][]{{-power, -power}, {-power, -power}});
            time = getRuntime();
            while (opModeIsActive() && getRuntime() - time < minTime)//In loop until there is glyph pn the robot
            {
                setMotorPower(new double[][]{{-power, -power}, {-power, -power}});
                intakeWithSensor(1);
                telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
                telemetry.update();

            }
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        intakeDC[0].setPower(-1);
        intakeDC[1].setPower(-1);
        sleep(125);
        intake(1);
        sleep(200);
        stopIntake();


        //second


        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            intakeWithSensor(1);
            Turn(20, 1);
        }
//        setMotorPower(new double[][]{{power, power}, {power, power}});
//
//        minTime = 0.3;
//        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
//            time = getRuntime();
//            while (opModeIsActive() && getRuntime() - time < minTime)//In loop until there is glyph pn the robot
//            {
//                setMotorPower(new double[][]{{power, power}, {power, power}});
//                intakeWithSensor(1);
//                telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
//                telemetry.update();
//
//            }
//            stopIntake();
//            setMotorPower(new double[][]{{0, 0}, {0, 0}});
//
//            setMotorPower(new double[][]{{-power, -power}, {-power, -power}});
//            time = getRuntime();
//            while (opModeIsActive() && getRuntime() - time < minTime)//In loop until there is glyph pn the robot
//            {
//                setMotorPower(new double[][]{{-power, -power}, {-power, -power}});
//                intakeWithSensor(1);
//                telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
//                telemetry.update();
//
//            }
//
////        intakeWithSensor(1);
//            telemetry.addLine("Finished loop2");
//            telemetry.update();
//            intakeWithSensor(1);
//
//            setMotorPower(new double[][]{{0, 0}, {0, 0}});
//
//            intakeDC[0].setPower(-1);
//            intakeDC[1].setPower(-1);
//            sleep(125);
//            intake(1);
//            sleep(200);
//            stopIntake();
//

        stopIntake();

        //}
    }

    public void getGlyphNear(double power) { //Drives to the Glyph pit and gathers until 2 glyphs
        //0= red,1= blue
        boolean degelCros = false;
        servoGlyphSides.setPosition(1); //open glyphs holder
        intakeWithSensor(1);
        double time = getRuntime();

        telemetry.addData("sensor Range rev", sensorColorDistanse.getDistance(DistanceUnit.CM));
        telemetry.update();
//        sleep(500);
        intakeWithSensor(1);
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)))
            Turn(-20, 1);
        setMotorPower(new double[][]{{power, power}, {power, power}});
        double minTime = 0.4;
        while (opModeIsActive() && Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)) && getRuntime() - time < minTime)//In loop until there is glyph pn the robot
        {
            setMotorPower(new double[][]{{power, power}, {power, power}});
            if (!degelCros && readVoltage() < voltageMin - 1.5) {

                setMotorPower(new double[][]{{0, 0}, {0, 0}});
                intakeDC[0].setPower(-1);
                intakeDC[1].setPower(-1);
                sleep(125);
                intake(0.9);
                sleep(120);

                degelCros = true;
                // stopIntake();
            }

            telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
        stopIntake();


//        intakeWithSensor(1);
        telemetry.addLine("Finished loop1");
        telemetry.update();
        intakeWithSensor(1);
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)))
            Turn(30, 1);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});

        intakeDC[0].setPower(-1);
        intakeDC[1].setPower(-1);
        sleep(125);
        setMotorPower(new double[][]{{power, power}, {power, power}});
        intake(1);
        sleep(120);
        stopIntake();
        intakeWithSensor(1);
        telemetry.addLine("Finished loop2");
        telemetry.update();


        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        stopIntake();

    }

    public void getMoreGilat(double power, int angle) { //Drives to the Glyph pit and gathers until 2 glyphs
        //0= red,1= blue
        double pidErr[] = {0, 0};
        servoGlyphSides.setPosition(1); //open glyphs holder
        double power1 = -0.7;
        double angle1 = -155;
        intake(1);
        double time = getRuntime();
        ResetHue(sensorColorBack, hsvValues);
        telemetry.addData("sensor Range rev", sensorColorDistanse.getDistance(DistanceUnit.CM));
        telemetry.update();
//        sleep(500);
        double distance = 25;
        while (opModeIsActive() && Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)) && getRuntime() - time < 0.7) {


        }


//        int timeDrive = 400;
//        while (opModeIsActive() && Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)) && getRuntime() - time < 2.3)//In loop until there is glyph pn the robot
//        {
//
////            pidErr = GyroPID(angle, pidErr[1]);
//            if (power1 > 0) {
////                distance += 20;
////                angle1 = -155;
//                timeDrive -= 200;
//            }
//            if (power1 < 0) {
////                distance -= 10;
////                angle1 = -30;
//                timeDrive += 300;
//            }
//            setMotorPower(new double[][]{{power1, power1}, {power1, power1}});
//            sleep(timeDrive);
////            driveByEncoder(distance, power1, angle1);
//            power1 = -power1;
//
//
//            telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
//            telemetry.update();
//
//        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        telemetry.addLine("Finished loop1");
        telemetry.update();
//        sleep(100);

        stopIntake();
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {//only if we dont have 2 glyph
            crossing(1, -0.7);
//            sleep(250);
            intake(1);
            pidErr = GyroPID(angle, pidErr[1]);
            setMotorPower(new double[][]{{-power + pidErr[0], -power - pidErr[0]}, {-power + pidErr[0], -power - pidErr[0]}});
            //setMotorPower(new double[][]{{-power, -power}, {-power, -power}});
            sleep(350);

            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            time = getRuntime();
            while (opModeIsActive() && (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) && getRuntime() - time < 0.4)//In loop until there is glyph pn the robot
            {
                setMotorPower(new double[][]{{power, power}, {power, power}});
                telemetry.addData("sensor Range rev", sensorColorDistanse.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            crossing(1, -0.7);
//            sleep(200);
            driveByEncoder(40, 1, 180);
            telemetry.addLine("Finished loop2");
            telemetry.update();
//            reverseIntake(1);

//            sleep(150);

        }
        stopIntake();
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }


    public void getMoreGlyph1(double power, int color, int angle, double time1) { //Drives to the Glyph pit and gathers until 2 glyphs
        //0= red,1= blue
        double pidErr[] = {0, 0};
        servoGlyphSides.setPosition(1); //open glyphs holder
        intakeDC[0].setPower(-1); //intake
        intakeDC[1].setPower(1);
        double time = getRuntime();
        ResetHue(sensorColorBack, hsvValues);
        telemetry.addData("sensor Range rev", sensorColorDistanse.getDistance(DistanceUnit.CM));
        telemetry.update();
//        sleep(500);
        while (opModeIsActive() && Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)) && getRuntime() - time < time1)//In loop until there is glyph pn the robot
        {
            intakeDC[0].setPower(-1); //intake
            intakeDC[1].setPower(1);
            pidErr = GyroPID(angle, pidErr[1]);
            setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
            //setMotorPower(new double[][]{{power, power}, {power, power}});
            telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
            telemetry.update();
            //  sleep(500);
            // drive to
        }
        //crossing
        intakeDC[0].setPower(-1);
        intakeDC[1].setPower(-1);
        sleep(150);
        while (opModeIsActive() && Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)) && getRuntime() - time < time1)//In loop until there is glyph pn the robot
        {
            intakeDC[0].setPower(-1); //intake
            intakeDC[1].setPower(1);
            pidErr = GyroPID(angle, pidErr[1]);
            setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
            //setMotorPower(new double[][]{{power, power}, {power, power}});
            telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
            telemetry.update();
            //  sleep(500);
            // drive to
        }
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            //crossing
            intakeDC[0].setPower(-1);
            intakeDC[1].setPower(-1);
            sleep(150);
        }
        if (sensorColorDistanse.getDistance(DistanceUnit.CM) > 0) {
            telemetry.addLine("NAN");
            telemetry.update();
            //  sleep(600);
            intakeDC[0].setPower(0); //Stop intake
            intakeDC[1].setPower(0);
            servoGlyphSides.setPosition(0);
            intakeSec = true;
        }
        telemetry.addLine("Finished loop1");
        telemetry.update();
//        sleep(100);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});


        if (!intakeSec) {
            intakeDC[0].setPower(-1);
            intakeDC[1].setPower(1);
        }


        if (!intakeSec) {
            intakeDC[0].setPower(-1);
            intakeDC[1].setPower(1);
        }
    }

    public void Lift() { // Use the intake motors and lift the glyphs door.
//Inside intake
        intake(1);

        sleep(80);
        servoGlyphSides.setPosition(0);
        stopIntake();

        double runTime = getRuntime();
        //Open door
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        while (opModeIsActive() && !touchUp.getState() && getRuntime() - runTime < 1.5) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);

        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
//        sleep(600);
//Reverse intake
        crossing(1, -0.7);
//        sleep(100);
        reverseIntake(1);
        sleep(500);
        stopIntake();
    }


    public void Centering(int color, double power) {

        Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                (int) (sensorColorBack.green() * SCALE_FACTOR),
                (int) (sensorColorBack.blue() * SCALE_FACTOR),
                hsvValues);
        Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                (int) (sensorColorLeft.green() * SCALE_FACTOR),
                (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                hsvValues1);
        if (color == 0) {
            while (hsvValues1[0] > 50 && opModeIsActive()) {
                setMotorPower(new double[][]{{power, -power}, {-power, power}});
                Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                        (int) (sensorColorLeft.green() * SCALE_FACTOR),
                        (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                        hsvValues1);
            }

            setMotorPower(new double[][]{{power, -power}, {-power, power}});
            sleep(50);
            Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                    (int) (sensorColorBack.green() * SCALE_FACTOR),
                    (int) (sensorColorBack.blue() * SCALE_FACTOR),
                    hsvValues);
            while (hsvValues[0] > 50 && hsvValues1[0] > 50 && opModeIsActive()) {
                setMotorPower(new double[][]{{power, -power}, {-power, power}});
                Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                        (int) (sensorColorLeft.green() * SCALE_FACTOR),
                        (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                        hsvValues1);
                Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                        (int) (sensorColorBack.green() * SCALE_FACTOR),
                        (int) (sensorColorBack.blue() * SCALE_FACTOR),
                        hsvValues);
            }


        }
        if (color == 1) {
            while (hsvValues1[0] < 110 && opModeIsActive()) {
                setMotorPower(new double[][]{{power, -power}, {-power, power}});
                Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                        (int) (sensorColorLeft.green() * SCALE_FACTOR),
                        (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                        hsvValues1);
            }

            setMotorPower(new double[][]{{-power, power}, {power, -power}});
            sleep(50);
            Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                    (int) (sensorColorBack.green() * SCALE_FACTOR),
                    (int) (sensorColorBack.blue() * SCALE_FACTOR),
                    hsvValues);
            while (hsvValues[0] > 50 && hsvValues1[0] > 50 && opModeIsActive()) {
                setMotorPower(new double[][]{{-power, power}, {power, -power}});
                Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                        (int) (sensorColorLeft.green() * SCALE_FACTOR),
                        (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                        hsvValues1);
                Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                        (int) (sensorColorBack.green() * SCALE_FACTOR),
                        (int) (sensorColorBack.blue() * SCALE_FACTOR),
                        hsvValues);
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
        if (hsvValues[0] != 0) {
            if (color == 0) { //Red case
                ResetHue(sensorColorBack, hsvValues);
                double tGlyph = getRuntime();
                if (hsvValues[0] > 200 || hsvValues[0] == 0) {
                    while (opModeIsActive() && (hsvValues[0] > 200 || hsvValues[0] == 0)) {//Until it spots red

//                    pidErr = GyroPID(0, pidErr[1]);
                        setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}}); //PID straight drive
                        telemetry.addLine("on the stone");
                        telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                        telemetry.addData("hsvValues[0]", hsvValues[0]);
                        telemetry.update();
                        ResetHue(sensorColorBack, hsvValues);
                    }
                }
            }
            if (color == 1) { //Blue case
                ResetHue(sensorColorBack, hsvValues);
                if (hsvValues[0] > 105) {
                    double tGlyph = getRuntime();
                    while (opModeIsActive() && hsvValues[0] > 105) {  //Until it spots blue

                        pidErr = GyroPID(0, pidErr[1]);
                        setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {0, 0}}); //PID straight drive
                        telemetry.addLine("InDriveOutofRed");
                        telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                        telemetry.addData("hsvValues[0]", hsvValues[0]);
                        telemetry.update();
                        ResetHue(sensorColorBack, hsvValues);
                    }
                }
            }
        } else {
            if (color == 0) { //Red case
                ResetHue(sensorColorLeft, hsvValues1);
                double tGlyph = getRuntime();
                if (hsvValues1[0] > 200 || hsvValues1[0] == 0) {
                    while (opModeIsActive() && (hsvValues1[0] > 200 || hsvValues1[0] == 0)) {//Until it spots red
//                        if (!touchUp.getState() && tGlyph + 1 > getRuntime()) {
//                            servoGlyphBring[0].setPosition(0);
//                            servoGlyphBring[1].setPosition(1);
//                        } else {
//                            servoGlyphBring[0].setPosition(0.5);
//                            servoGlyphBring[1].setPosition(0.5);
//                        }
//                    pidErr = GyroPID(0, pidErr[1]);
                        setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}}); //PID straight drive
                        telemetry.addLine("on the stone");
                        telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                        telemetry.addData("hsvValues[0]", hsvValues1[0]);
                        telemetry.update();
                        ResetHue(sensorColorLeft, hsvValues1);
                    }
                }
            }
            if (color == 1) { //Blue case
                ResetHue(sensorColorLeft, hsvValues1);
                if (hsvValues1[0] > 105) {
                    double tGlyph = getRuntime();
                    while (opModeIsActive() && hsvValues1[0] > 105) {  //Until it spots blue
//                        if (!touchUp.getState() && tGlyph + 1 > getRuntime()) {
//                            servoGlyphBring[0].setPosition(0);
//                            servoGlyphBring[1].setPosition(1);
//                        } else {
//                            servoGlyphBring[0].setPosition(0.5);
//                            servoGlyphBring[1].setPosition(0.5);
//                        }
                        pidErr = GyroPID(0, pidErr[1]);
                        setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {0, 0}}); //PID straight drive
                        telemetry.addLine("InDriveOutofRed");
                        telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                        telemetry.addData("hsvValues[0]", hsvValues[0]);
                        telemetry.update();
                        ResetHue(sensorColorLeft, hsvValues1);
                    }
                }


            }
        }
        telemetry.addLine("Finish Get down with color");
        telemetry.update();
        setMotorPower(new double[][]{{0, 0}, {0, 0}});

    }


    //    public void getDownColor() {
//        Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
//                (int) (sensorColorBack.green() * SCALE_FACTOR),
//                (int) (sensorColorBack.blue() * SCALE_FACTOR),
//                hsvValues);
//        double pidErr[] = {0, 0};
//        telemetry.addData("hsvValues[0]", hsvValues[0]);
//        telemetry.update();
//        if (color == 0 && opModeIsActive()) {
//            double time = getRuntime();
//            while (opModeIsActive() && hsvValues[0] > redColor && (time + 2 > getRuntime())) {
//                Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
//                        (int) (sensorColorBack.green() * SCALE_FACTOR),
//                        (int) (sensorColorBack.blue() * SCALE_FACTOR),
//                        hsvValues);
//                pidErr = GyroPID(heading, pidErr[1]);
//                setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
//                telemetry.addLine("ontheGrey");
//                telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
//                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//                telemetry.addData("hsvValues[0]", hsvValues[0]);
//                telemetry.update();
//
//            }
//        }
    public void driveByColorWith2Sensors(int color, double heading, double power, double direction)//0=red, blue=1
    {
        double deg2rad = direction / 180 * Math.PI;


        double dcmotorColor[][] = new double[3][3];
        dcmotorColor[0][0] = ((Math.cos(deg2rad) + Math.sin(deg2rad)));
        dcmotorColor[1][0] = ((Math.cos(deg2rad) - Math.sin(deg2rad)));
        dcmotorColor[0][1] = ((Math.cos(deg2rad) - Math.sin(deg2rad)));
        dcmotorColor[1][1] = ((Math.cos(deg2rad) + Math.sin(deg2rad)));
        Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                (int) (sensorColorBack.green() * SCALE_FACTOR),
                (int) (sensorColorBack.blue() * SCALE_FACTOR),
                hsvValues);
        Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                (int) (sensorColorLeft.green() * SCALE_FACTOR),
                (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                hsvValues1);
        double pidErr[] = {0, 0};
        telemetry.addData("hsvValues[0]", hsvValues[0]);
        telemetry.addData("hsvValues1[0]", hsvValues1[0]);
        telemetry.update();
        if (color == 0 && opModeIsActive()) {
            double time = getRuntime();
            while (opModeIsActive() && hsvValues[0] > redColorRightSensor && hsvValues1[0] > redColorLeftSensor && hsvValues[0] != 0 && hsvValues1[0] != 0 && (time + 3 > getRuntime())) {
                Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                        (int) (sensorColorBack.green() * SCALE_FACTOR),
                        (int) (sensorColorBack.blue() * SCALE_FACTOR),
                        hsvValues);
                Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                        (int) (sensorColorLeft.green() * SCALE_FACTOR),
                        (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                        hsvValues1);
                pidErr = GyroPID(heading, pidErr[1]);
                setMotorPower(new double[][]{{
                        (power * (Math.cos(deg2rad) + Math.sin(deg2rad) - pidErr[0])),
                        (power * (Math.cos(deg2rad) - Math.sin(deg2rad) + pidErr[0]))}, {
                        (power * (Math.cos(deg2rad) - Math.sin(deg2rad)) - pidErr[0]),
                        (power * (Math.cos(deg2rad) + Math.sin(deg2rad)) + pidErr[0])}
                });
                telemetry.addLine("on the Grey");
                telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("hsvValues[0]", hsvValues[0]);
                telemetry.addData("hsvValues1[0]", hsvValues1[0]);
                telemetry.update();

            }
            double time2 = getRuntime();

        }
        if (color == 1 && opModeIsActive()) {
            double time1 = getRuntime();

            while (opModeIsActive() && hsvValues[0] != 0 && hsvValues1[0] != 0 && hsvValues[0] < blueColorRightSensor && hsvValues1[0] < blueColorLeftSensor && (time1 + 2 > getRuntime())) {
                Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                        (int) (sensorColorBack.green() * SCALE_FACTOR),
                        (int) (sensorColorBack.blue() * SCALE_FACTOR),
                        hsvValues);
                Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                        (int) (sensorColorLeft.green() * SCALE_FACTOR),
                        (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                        hsvValues1);
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
        telemetry.addLine("finish drive");
        telemetry.update();

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
            while (opModeIsActive() && hsvValues[0] > redColorRightSensor && (time + 2 > getRuntime())) {
                Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                        (int) (sensorColorBack.green() * SCALE_FACTOR),
                        (int) (sensorColorBack.blue() * SCALE_FACTOR),
                        hsvValues);
                pidErr = GyroPID(heading, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
                telemetry.addLine("ontheGrey");
                telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("hsvValues[0]", hsvValues[0]);
                telemetry.update();

            }
            double time2 = getRuntime();
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
        }
        if (color == 1 && opModeIsActive()) {
            double time1 = getRuntime();

            while (opModeIsActive() && hsvValues[0] < blueColorRightSensor && (time1 + 2 > getRuntime())) {
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


    }

    public void starfeByColor(int color, double heading, double power)//0=red, blue=1
    {

        ResetHue(sensorColorBack, hsvValues);
        double pidErr[] = {0, 0};
        telemetry.addData("hsvValues[0]", hsvValues[0]);
        telemetry.update();
        if (color == 0 && opModeIsActive()) { //Red case
            double time = getRuntime();
            while (opModeIsActive() && hsvValues[0] > redColorRightSensor && (time + 1.5 > getRuntime())) { //Either it is spots red or 1.5 seconds passed
                ResetHue(sensorColorBack, hsvValues);
                pidErr = GyroPID(heading, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
                telemetry.addLine("ontheGrey");
                telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("hsvValues[0]", hsvValues[0]);
                telemetry.update();

            }
            double time2 = getRuntime();
            while (opModeIsActive() && hsvValues[0] > 76 && (time2 + 1.3 > getRuntime())) {
                ResetHue(sensorColorBack, hsvValues);
                pidErr = GyroPID(heading, pidErr[1]);
                setMotorPower(new double[][]{{-power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], -power + pidErr[0]}});
                telemetry.addLine("InDriveRed");
                telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("hsvValues[0]", hsvValues[0]);
                telemetry.update();

            }
            if (color == 1 && opModeIsActive()) { //Blue case
                double time1 = getRuntime();

                while (opModeIsActive() && hsvValues[0] < blueColorRightSensor && (time1 + 1.5 > getRuntime())) {
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
            sleep(300);
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
        }
    }

//    public void DriveByDistanceLift(double distance, double power, ModernRoboticsI2cRangeSensor
//            rangeSensor, double heading, int correct) {
//        double pidErr[] = {0, 0}; //PID reset
//        double reverseIntake = 0, crosing = 0, intake = 0;
//        range0 = true; //in case the ranaesensor is detacahed range0 is false
//        int lift = 1;
//        for (int ii = 0; ii < correct + 1; ii++) {
//            if (ii > 0)
//                power *= 0.8;
//            if (distance > rangeSensor.getDistance(DistanceUnit.CM)) {// Further then tareget distance
//                telemetry.addData("HERE", rangeSensor.getDistance((DistanceUnit.CM)));
//                telemetry.update();
//
//                double time = getRuntime();
//                while (rangeSensor.getDistance(DistanceUnit.CM) != 0 && opModeIsActive() && (rangeSensor.getDistance(DistanceUnit.CM) + 2 < distance) && getRuntime() < (time + distance / 10)) {
//                    pidErr = GyroPID(heading, pidErr[1]);
//                    setMotorPower(new double[][]{{-power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], -power + pidErr[0]}});
//                    telemetry.addData("FromWall", rangeSensor.getDistance((DistanceUnit.CM)));
//                    telemetry.update();
//
//                    if (lift == 1) {
//
//
//                        lift += 1;
//                    }
//                    if (lift == 2) {
//                        servoGlyphSides.setPosition(0);
//                        stopIntake();
//                        lift += 1;
//                    }
//                    if (lift == 3) {
//                        double runTime = getRuntime();
//                        //Open door
//                        while (opModeIsActive() && !touchUp.getState() && getRuntime() - runTime < 1.5) {
//                            servoGlyphBring[0].setPosition(0);
//                            servoGlyphBring[1].setPosition(1);
//
//                        }
//                        servoGlyphBring[0].setPosition(0.5);
//                        servoGlyphBring[1].setPosition(0.5);
//                        lift += 1;
//                    }
//                    if (lift == 4) {
//                        crosing = getRuntime();
//
//
//                        lift += 1;
//                    }
//                    if (lift == 5) {
//                        reverseIntake = getRuntime();
//                        stopIntake();
//
//                    }
//                    if (intake + 80 > getRuntime()) {
//                        intake(1);
//
//                    } else stopIntake();
//                    if (crosing + 100 > getRuntime()) {
//                        crosing(1);
//                    } else
//                        stopIntake();
//                    if (reverseIntake + 500 > getRuntime() && lift == 5) {
//                        reverseIntake(1);
//                        lift += 1;
//                    } else
//                        stopIntake();
//                }
//            } else if ((distance) < rangeSensor.getDistance(DistanceUnit.CM)) {//Closer then tareget distance
//                double time = getRuntime();
//
//                while (rangeSensor.getDistance(DistanceUnit.CM) != 0 && opModeIsActive() && (distance < rangeSensor.getDistance(DistanceUnit.CM) - 2) && getRuntime() < (time + distance / 10)) {
//                    pidErr = GyroPID(heading, pidErr[1]);
//                    setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
//                    telemetry.addData("ToWall", rangeSensor.getDistance((DistanceUnit.CM)));
//                    telemetry.update();
//                    if (lift == 1) {
//                        intake(1);
//                        sleep(80);
//                        stopIntake();
//                        lift += 1;
//                    }
//                    if (lift == 2) {
//                        servoGlyphSides.setPosition(0);
//                        stopIntake();
//                        lift += 1;
//                    }
//                    if (lift == 3) {
//                        double runTime = getRuntime();
//                        //Open door
//                        while (opModeIsActive() && !touchUp.getState() && getRuntime() - runTime < 1.5) {
//                            servoGlyphBring[0].setPosition(0);
//                            servoGlyphBring[1].setPosition(1);
//
//                        }
//                        servoGlyphBring[0].setPosition(0.5);
//                        servoGlyphBring[1].setPosition(0.5);
//                        lift += 1;
//                    }
//                    if (lift == 4) {
//                        crosing(1);
//                        sleep(100);
//                        stopIntake();
//                        lift += 1;
//                    }
//                    if (lift == 5) {
//                        reverseIntake(1);
//                        reverseIntake = getRuntime();
//                        stopIntake();
//                        lift += 1;
//                    }
//                    if (reverseIntake + 500 > getRuntime() && lift == 5) {
//                        reverseIntake(1);
//                    } else
//                        stopIntake();
//                }
//
//
//            }
//            setMotorPower(new double[][]{{0, 0}, {0, 0}});  //stop drivetrain motors
//            if (rangeSensor.getDistance(DistanceUnit.CM) == 0) { //In case the range sensor is detached.
//                range0 = false;
//            }
//        }
//    }

    public void DriveByDistance(double distance, double power, ModernRoboticsI2cRangeSensor
            rangeSensor, double heading, int correct) {
        double pidErr[] = {0, 0}; //PID reset
        range0 = true; //in case the ranaesensor is detacahed range0 is false
        for (int ii = 0; ii < correct + 1; ii++) {
            if (ii > 0)
                power *= 0.75;
            if (distance > rangeSensor.getDistance(DistanceUnit.CM)) {// Further then tareget distance
                telemetry.addData("HERE", rangeSensor.getDistance((DistanceUnit.CM)));                     
                telemetry.update();
                double time = getRuntime();
                while (rangeSensor.getDistance(DistanceUnit.CM) != 0 && opModeIsActive() && (rangeSensor.getDistance(DistanceUnit.CM) + 2 < distance) && getRuntime() < (time + distance / 10)) {
                    //     pidErr = GyroPID(heading, pidErr[1]);
                    setMotorPower(new double[][]{{-power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], -power + pidErr[0]}});
                    telemetry.addData("FromWall", rangeSensor.getDistance((DistanceUnit.CM)));
                    telemetry.update();
//                sleep(200);
                }
            } else if ((distance) < rangeSensor.getDistance(DistanceUnit.CM)) {//Closer then tareget distance
                double time = getRuntime();

                while (rangeSensor.getDistance(DistanceUnit.CM) != 0 && opModeIsActive() && (distance < rangeSensor.getDistance(DistanceUnit.CM) - 2) && getRuntime() < (time + distance / 10)) {
                    //  pidErr = GyroPID(heading, pidErr[1]);
                    setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});
                    telemetry.addData("ToWall", rangeSensor.getDistance((DistanceUnit.CM)));
                telemetry.update();
            }

        }
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});  //stop drivetrain motors
        if (rangeSensor.getDistance(DistanceUnit.CM) == 0) { //In case the range sensor is detached.
            range0 = false;

        }
    }

    public void DriveByDistance2(double distance, double power, ModernRoboticsI2cRangeSensor
            rangeSensor, double heading, int correct) {
        double pidErr[] = {0, 0}; //PID reset
        range0 = true; //in case the ranaesensor is detacahed range0 is false
        for (int ii = 0; ii < correct + 1; ii++) {
            if (ii > 0)
                power *= 0.8;
            if (distance > rangeSensor.getDistance(DistanceUnit.CM)) {// Further then tareget distance
                telemetry.addData("HERE", rangeSensor.getDistance((DistanceUnit.CM)));
                telemetry.update();
                double time = getRuntime();
                while (rangeSensor.getDistance(DistanceUnit.CM) != 0 && opModeIsActive() && (rangeSensor.getDistance(DistanceUnit.CM) + 2 < distance) && getRuntime() < (time + distance / 10)) {
                    //     pidErr = GyroPID(heading, pidErr[1]);
                    setMotorPower(new double[][]{{-power - pidErr[0], -power + pidErr[0]}, {-0 - pidErr[0], -0 + pidErr[0]}});
                    telemetry.addData("FromWall", rangeSensor.getDistance((DistanceUnit.CM)));
                    telemetry.update();
//                sleep(200);
                }
            } else if ((distance) < rangeSensor.getDistance(DistanceUnit.CM)) {//Closer then tareget distance
                double time = getRuntime();

                while (rangeSensor.getDistance(DistanceUnit.CM) != 0 && opModeIsActive() && (distance < rangeSensor.getDistance(DistanceUnit.CM) - 2) && getRuntime() < (time + distance / 10)) {
                    //  pidErr = GyroPID(heading, pidErr[1]);
                    setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {0 - pidErr[0], 0 + pidErr[0]}});
                    telemetry.addData("ToWall", rangeSensor.getDistance((DistanceUnit.CM)));
                    telemetry.update();
                }

            }
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});  //stop drivetrain motors
        if (rangeSensor.getDistance(DistanceUnit.CM) == 0) { //In case the range sensor is detached.
            range0 = false;
        }
        //sleep(nSleep);
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


    public void scoreGlyph(boolean intake) {
        telemetry.addLine("in Score Glyph");
        telemetry.update();
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        if (intake) {
            servoGlyphSides.setPosition(1);
            intake(1);
            sleep(200);
            servoGlyphSides.setPosition(0);
            stopIntake();
            sleep(200);
        }

        double time5 = getRuntime();
        while (opModeIsActive() && !touchUp.getState() && getRuntime() - time5 < 1.5) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);

        driveByEncoder(4, 0.7, 0);//go to the criptobox
        for (int i = 0; i < 2; i++) {
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(150);
            setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
            sleep(150);
        }
        setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
        sleep(150);
        driveByEncoder(6, 0.7, 180);
        time5 = getRuntime();
        while (opModeIsActive() && !touchUp.getState() && getRuntime() - time5 < 1.5) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
        servoGlyphSides.setPosition(0.4);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        driveByEncoder(4, 0.7, 0);
        servoGlyphSides.setPosition(0.4);
        sleep(250);
        // sleep(200);
//        driveByEncoder(4, 1, 180);
        // sleep(200);

        driveByEncoder(13, 0.4, 180);
//        setMotorPower(new double[][]{{-1, -1}, {-1, -1}});
//        sleep(200);

//        servoGlyphSides.setPosition(0);
//        driveByEncoder(6, 1, 180);

//        setMotorPower(new double[][]{{0.6, 0.6}, {0.6, 0.6}});


        telemetry.addLine("in Push");

//        NewdriveRobotEncoder(10, 0, 0.5, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        driveByEncoder(9, 0.6, 0);
        if (runtime.seconds() >= 29) {
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            sleep(1000);
        }
        double rangedi = range.getDistance(DistanceUnit.CM) + 12;
        DistanceAndDownDoor(rangedi, -0.6);
//        DriveByDistance(rangedi, 1, range, 0, 0);
        double runTime = getRuntime();
        while (opModeIsActive() && !touchDown.getState() && getRuntime() - runTime < 1.5) {
            servoGlyphBring[0].setPosition(1);
            servoGlyphBring[1].setPosition(0);


        }
        if (touchDown.getState() || !magnetDown.getState()) {
            servoGlyphSides.setPosition(1);
        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
//        driveByEncoder(22, 1, 180);


        telemetry.addLine("in Go Back");
//
//        NewdriveRobotEncoder(5, 0, -0.38, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


    }

    public void scoreGlyphNear(boolean intake, int direction) {
        telemetry.addLine("in Score Glyph");
        telemetry.update();
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        if (intake) {
            servoGlyphSides.setPosition(0.6);
//            intake(1);
//            sleep(100);
            intakeDC[0].setPower(-1);
            intakeDC[1].setPower(-1);
            sleep(125);
            intake(1);
            sleep(500);
            stopIntake();
            servoGlyphSides.setPosition(0.05);
//            stopIntake();
            sleep(350);
        } else
            servoGlyphSides.setPosition(0);

        double time5 = getRuntime();
        while (opModeIsActive() && getRuntime() - time5 < 0.2) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }

        time5 = getRuntime();
        while (opModeIsActive() && !touchUp.getState() && getRuntime() - time5 < 2) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);

        stopIntake();
        while (opModeIsActive() && !touchUp.getState() && getRuntime() - time5 < 2) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }

        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
//        driveByEncoder(4, 0.7, 0);//go to the criptobox
        if (direction == 1) {
            setMotorPower(new double[][]{{0, 0.8}, {0, 0.8}});
            sleep(200);
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(200);
            setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
            sleep(100);
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(100);
        }
        if (direction == 2) {
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(200);
            setMotorPower(new double[][]{{0, 0.8}, {0, 0.8}});
            sleep(200);
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(100);
            setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
            sleep(100);

        }

        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        driveByEncoder(6, 0.7, 180);


        servoGlyphSides.setPosition(0.4);
        sleep(200);
        time5 = getRuntime();
        while (opModeIsActive() && getRuntime() - time5 < 0.1) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
        setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
        sleep(50);
        setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
        sleep(50);

        setMotorPower(new double[][]{{0, 0}, {0, 0}});
//        driveByEncoder(3, 0.4, 0);
        driveByEncoder(8, 0.6, 180);


        telemetry.addLine("in Push");


        driveByEncoder(8, 0.4, 0);
//        if (getRuntime() >= 29) {
//            setMotorPower(new double[][]{{0, 0}, {0, 0}});
//            sleep(1000);
        servoGlyphSides.setPosition(0.3);
        driveByEncoder(8, 0.4, 180);
        double rangedi = range.getDistance(DistanceUnit.CM) + 8;
        DistanceAndDownDoor(rangedi, -0.6);
        double runTime = getRuntime();
        while (opModeIsActive() && !touchDown.getState() && getRuntime() - runTime < 1.5) {
            servoGlyphBring[0].setPosition(1);
            servoGlyphBring[1].setPosition(0);


        }
        if (touchDown.getState() || !magnetDown.getState()) {
            servoGlyphSides.setPosition(1);
        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
//        driveByEncoder(22, 1, 180);


        telemetry.addLine("in Go Back");
//
//        NewdriveRobotEncoder(5, 0, -0.38, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


    }

    public void scoreGlyphFar(boolean intake, int direction) {
        telemetry.addLine("in Score Glyph");
        telemetry.update();
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        if (intake) {
            servoGlyphSides.setPosition(0.6);
//            intake(1);
//            sleep(100);
            intakeDC[0].setPower(-1);
            intakeDC[1].setPower(-1);
            sleep(125);
            intake(1);
            sleep(500);
            stopIntake();
            servoGlyphSides.setPosition(0.05);
//            stopIntake();
            sleep(350);
        } else
            servoGlyphSides.setPosition(0);

        double time5 = getRuntime();
        while (opModeIsActive() && getRuntime() - time5 < 0.2) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }

        time5 = getRuntime();
        while (opModeIsActive() && !touchUp.getState() && getRuntime() - time5 < 2) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);

        stopIntake();
        while (opModeIsActive() && !touchUp.getState() && getRuntime() - time5 < 2) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }

        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
//        driveByEncoder(4, 0.7, 0);//go to the criptobox
        if (direction == 1) {
            setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
            sleep(150);
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(150);
            setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
            sleep(100);
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(100);
        }
        if (direction == 2) {
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(100);
            setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
            sleep(100);
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(100);
            setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
            sleep(100);

        }

        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        driveByEncoder(6, 0.7, 180);


        servoGlyphSides.setPosition(0.4);
        sleep(200);
        time5 = getRuntime();
        while (opModeIsActive() && getRuntime() - time5 < 0.1) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
        setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
        sleep(50);
        setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
        sleep(50);

        setMotorPower(new double[][]{{0, 0}, {0, 0}});
//        driveByEncoder(3, 0.4, 0);
        driveByEncoder(8, 0.6, 180);


        telemetry.addLine("in Push");


        driveByEncoder(8, 0.4, 0);
//        if (getRuntime() >= 29) {
//            setMotorPower(new double[][]{{0, 0}, {0, 0}});
//            sleep(1000);
        servoGlyphSides.setPosition(0.3);
        driveByEncoder(8, 0.4, 180);
        double rangedi = range.getDistance(DistanceUnit.CM) + 8;
        DistanceAndDownDoor(rangedi, -0.6);
        double runTime = getRuntime();
        while (opModeIsActive() && !touchDown.getState() && getRuntime() - runTime < 1.5) {
            servoGlyphBring[0].setPosition(1);
            servoGlyphBring[1].setPosition(0);


        }
        if (touchDown.getState() || !magnetDown.getState()) {
            servoGlyphSides.setPosition(1);
        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
//        driveByEncoder(22, 1, 180);


        telemetry.addLine("in Go Back");
//
//        NewdriveRobotEncoder(5, 0, -0.38, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


    }

    public void scoreGlyphFar2(boolean intake, int direction) {
        telemetry.addLine("in Score Glyph");
        telemetry.update();
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        if (intake) {
            servoGlyphSides.setPosition(0.6);
//            intake(1);
//            sleep(100);
            intakeDC[0].setPower(-1);
            intakeDC[1].setPower(-1);
            sleep(125);
            intake(1);
            sleep(500);
            stopIntake();
            servoGlyphSides.setPosition(0.05);
//            stopIntake();
            sleep(350);
        } else
            servoGlyphSides.setPosition(0);

        double time5 = getRuntime();
        while (opModeIsActive() && getRuntime() - time5 < 0.2) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }

        time5 = getRuntime();
        while (opModeIsActive() && !touchUp.getState() && getRuntime() - time5 < 2) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }
        servoGlyphBring[ 0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);

        stopIntake();
        while (opModeIsActive() && !touchUp.getState() && getRuntime() - time5 < 2) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }

        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
//        driveByEncoder(4, 0.7, 0);//go to the criptobox
        if (direction == 1) {
            setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
            sleep(150);
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(150);
            setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
            sleep(100);
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(100);
        }


        if (direction == 2) {
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(150);
            setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
            sleep(150);
            setMotorPower(new double[][]{{0.7, 0}, {0.7, 0}});
            sleep(100);
            setMotorPower(new double[][]{{0, 0.7}, {0, 0.7}});
            sleep(100);

        }

        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        //  driveByEncoder(8, 0.7, 180);


        servoGlyphSides.setPosition(0.4);
        sleep(200);
        time5 = getRuntime();
        while (opModeIsActive() && getRuntime() - time5 < 0.1) {
            servoGlyphBring[0].setPosition(0);
            servoGlyphBring[1].setPosition(1);
        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);


        setMotorPower(new double[][]{{0, 0}, {0, 0}});
//        driveByEncoder(3, 0.4, 0);
        driveByEncoder(8, 0.6, 180);


        driveByEncoder(12, 0.4, 0);
//        if (getRuntime() >= 29) {
//            setMotorPower(new double[][]{{0, 0}, {0, 0}});
//            sleep(1000);
        servoGlyphSides.setPosition(0.3);
        driveByEncoder(12, 0.4, 180);
//        double rangedi = range.getDistance(DistanceUnit.CM) + 8;
//        DistanceAndDownDoor(rangedi, -0.6);
        double runTime = getRuntime();
        while (opModeIsActive() && !touchDown.getState() && getRuntime() - runTime < 1.5) {
            servoGlyphBring[0].setPosition(1);
            servoGlyphBring[1].setPosition(0);


        }
        if (touchDown.getState() || !magnetDown.getState()) {
            servoGlyphSides.setPosition(1);
        }
        servoGlyphBring[0].setPosition(0.5);
        servoGlyphBring[1].setPosition(0.5);
//        driveByEncoder(22, 1, 180);


        telemetry.addLine("in Go Back");
//
//        NewdriveRobotEncoder(5, 0, -0.38, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


    }

    public void driveByEncoder(double goalDist, double k, double direction) {// Drive by encoders and converts incoders ticks to distance in cm and drives until distance is completed.
//        intakeWithSensor(1);
//dc motor [0][0] not working
        // k is power for motors
        //Reset encoders
        double deg2rad = direction / 180 * Math.PI;
        double d = (goalDist * tixRound) / cmRound;


        double currEncoder[][] = new double[2][2];
        double goalEncoder[][] = new double[2][2];
        double dEncoder[][] = new double[2][2];
        dEncoder[0][0] = (d * (Math.cos(deg2rad) + Math.sin(deg2rad)));
        dEncoder[1][0] = (d * (Math.cos(deg2rad) - Math.sin(deg2rad)));
        dEncoder[0][1] = (d * (Math.cos(deg2rad) - Math.sin(deg2rad)));
        dEncoder[1][1] = (d * (Math.cos(deg2rad) + Math.sin(deg2rad)));
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
//                currEncoder[i][j] = dcmotor[i][j].getCurrentPosition();
                goalEncoder[i][j] = dcmotor[i][j].getCurrentPosition() + dEncoder[i][j];
                // dcmotor[i][j].setDirection(DcMotorSimple.Direction.REVERSE);

                dcmotor[i][j].setTargetPosition((int) (goalEncoder[i][j]));
                dcmotor[i][j].setMode(DcMotor.RunMode.RUN_TO_POSITION);


            }
        }
//        intakeWithSensor(1);
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                dcmotor[i][j].setPower(k);


        double err = 200;
        while (err > 100 && opModeIsActive()) {
            err = 0;
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++) {
                    err += Math.abs(goalEncoder[i][j] - dcmotor[i][j].getCurrentPosition());
                    telemetry.addData(" encoder", dcmotor[i][j].getCurrentPosition());
                }
            err /= 4;

            telemetry.addData(" err", err);
            telemetry.update();
        }
//        intakeWithSensor(1);
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                dcmotor[i][j].setPower(0);
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
//                currEncoder[i][j] = dcmotor[i][j].getCurrentPosition();
//                goalEncoder[i][j] = currEncoder[i][j] + dEncoder;
            {
                dcmotor[i][j].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                dcmotor[i][j].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

        dcmotor[0][0].setDirection(DcMotorSimple.Direction.REVERSE);
        dcmotor[0][1].setDirection(DcMotorSimple.Direction.FORWARD);
        dcmotor[1][0].setDirection(DcMotorSimple.Direction.REVERSE);
        dcmotor[1][1].setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void driveByEncoder2(double goalDist, double k, double direction) {// Drive by encoders and converts incoders ticks to distance in cm and drives until distance is completed.

//dc motor [0][0] not working
        // k is power for motors
        //Reset encoders
        double deg2rad = direction / 180 * Math.PI;
        double d = (goalDist * tixRound) / cmRound;


        double currEncoder[][] = new double[2][2];
        double goalEncoder[][] = new double[2][2];
        double dEncoder[][] = new double[2][2];
        dEncoder[0][0] = (d * (Math.cos(deg2rad) + Math.sin(deg2rad)));
        //dEncoder[1][0] = (d * (Math.cos(deg2rad) - Math.sin(deg2rad)));
        dEncoder[0][1] = (d * (Math.cos(deg2rad) - Math.sin(deg2rad)));
        //dEncoder[1][1] = (d * (Math.cos(deg2rad) + Math.sin(deg2rad)));
        int i = 0;
        for (int j = 0; j < 2; j++) {
//                currEncoder[i][j] = dcmotor[i][j].getCurrentPosition();
            goalEncoder[i][j] = dcmotor[i][j].getCurrentPosition() + dEncoder[i][j];
            // dcmotor[i][j].setDirection(DcMotorSimple.Direction.REVERSE);

            dcmotor[i][j].setTargetPosition((int) (goalEncoder[i][j]));
            dcmotor[i][j].setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }


        for (int j = 0; j < 2; j++)
            dcmotor[i][j].setPower(k);


        double err = 200;
        while (err > 100 && opModeIsActive()) {
            err = 0;
            for (int j = 0; j < 2; j++) {
                err += Math.abs(goalEncoder[i][j] - dcmotor[i][j].getCurrentPosition());
                telemetry.addData(" encoder", dcmotor[i][j].getCurrentPosition());
            }
            err /= 4;

            telemetry.addData(" err", err);
            telemetry.update();
        }
        for (int j = 0; j < 2; j++)
            dcmotor[i][j].setPower(0);
        for (int j = 0; j < 2; j++)
//                currEncoder[i][j] = dcmotor[i][j].getCurrentPosition();
//                goalEncoder[i][j] = currEncoder[i][j] + dEncoder;
        {
            dcmotor[i][j].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[i][j].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        dcmotor[0][0].setDirection(DcMotorSimple.Direction.REVERSE);
        dcmotor[0][1].setDirection(DcMotorSimple.Direction.FORWARD);
        dcmotor[1][0].setDirection(DcMotorSimple.Direction.REVERSE);
        dcmotor[1][1].setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void NewdriveRobotEncoder(double goalDist, double direction, double k,
                                     double heading) {// Drive by encoders and converts incoders ticks to distance in cm and drives until distance is completed.
        if (opModeIsActive()) {
//dc motor [0][0] not working
            // k is power for motors
            //Reset encoders
            dcmotor[1][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[1][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcmotor[0][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[0][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcmotor[1][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[1][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double pidErr[] = {0, 0};
//PID reset
            pidErr = GyroPID(heading, pidErr[1]); //Convert degrees to radians
            double deg2rad = direction / 180 * Math.PI;
            double currDist = 0;
            double err = goalDist;
            double time0 = getRuntime();
// Drive until distance target completed

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

    public void encoderAndCrosing(double goalDist, double direction, double k,
                                  double heading) {// Drive by encoders and converts incoders ticks to distance in cm and drives until distance is completed.
        if (opModeIsActive()) {
//dc motor [0][0] not working
            // k is power for motors
            //Reset encoders
            dcmotor[1][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[1][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcmotor[0][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[0][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcmotor[1][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[1][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double pidErr[] = {0, 0};
//PID reset
            pidErr = GyroPID(heading, pidErr[1]); //Convert degrees to radians
            double deg2rad = direction / 180 * Math.PI;
            double currDist = 0;
            double err = goalDist;
            double time0 = getRuntime();
// Drive until distance target completed

            while (opModeIsActive() && (err) > 2 && (getRuntime() - time0) < 0.15) {
                intakeDC[0].setPower(-1);
                intakeDC[1].setPower(-1);
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

            }
            time0 = getRuntime();
            while (opModeIsActive() && (err) > 2 && (getRuntime() - time0) < 0.2) {
                intake(1);
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

            }
            stopIntake();
            time0 = getRuntime();
            while (opModeIsActive() && (err) > 2 && (getRuntime() - time0) < (goalDist / 10)) {

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

            }
        }

        setMotorPower(new double[][]{{0.0, 0.0}, {0.0, 0.0}});

    }


    public void newTurn(double angle, double power) {
        double newAngle = getIntegratedZAxis() + angle;
        double pidErr[] = {0, angle};
        //setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});

        while (Math.abs(pidErr[1]) > 1 && opModeIsActive()) {
            telemetry.addData("roll", imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
            pidErr = GyroPID(newAngle, pidErr[1]);
            if (Math.abs(pidErr[0]) < 0.2)
                pidErr[0] += Math.signum(pidErr[0]) * 0.15;
            setMotorPower(new double[][]{{-pidErr[0], pidErr[0]}, {-pidErr[0], pidErr[0]}});
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }


    public double[] GyroPID(double heading, double lasterror) {
        double kp = 0.015, kd = 0.01, ki = 0, nexterror = 0;
        double err = heading - imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (err > 180)
            err = err - 360;
        while (err < -180)
            err = err + 360;
        lasterror = err - lasterror;
        double pd = nexterror * ki + lasterror * kd + err * kp;
        return (new double[]{-pd, err});
    }

    public void Turn(double angle, double pow) {
        telemetry.addLine("IN TURN:");
        telemetry.update();
        double angle0 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double target = angle0 + angle;
        boolean flag1 = false;
        if (angle < 0) {
            if (target < -180)
                flag1 = true;
            setMotorPower(new double[][]{{-pow, pow}, {-pow, pow}});
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
                double power = 0.2 * (-angle1 + target) / 15 + 0.3;
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
            setMotorPower(new double[][]{{+pow, -pow}, {+pow, -pow}});
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
                double power = 0.2 * (-angle1 + target) / 15 + 0.3;
                setMotorPower(new double[][]{{pow, -pow}, {pow, -pow}});
                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (flag1)
                    angle1 += 360;
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
        }
    }


    public void setMotorPower(double[][] power) { //Stores the four drivetrain motors power in array
        for (int row = 0; opModeIsActive() && row < 2; row++)
            for (int col = 0; opModeIsActive() && col < 2; col++)
                dcmotor[row][col].setPower(power[row][col]);
    }


    public double readVoltage1() {
        double result = Double.POSITIVE_INFINITY;
        int i = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            ++i;
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }

            telemetry.addData("voltage: ", result);
            telemetry.update();
        }
        //      telemetry.addData("voltage: ", result);
        telemetry.update();
        return result;

    }

    public void colDrive(ModernRoboticsI2cRangeSensor range, int direction, double power,
                         int colVu) {//1= red , 2=blue
        // get reference rangea
        double dist = 25;


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


        pidErr[1] = 0;
        double dist2 = 6;
        double dist3 = 5;
        double range11 = 0;
        for (int sumCol = 0; sumCol <= colVu /*+ 1*/; sumCol++) {
            double range00 = range.getDistance(DistanceUnit.CM);

            power = 0.6;
            setMotorPower(new double[][]{{power, -power}, {-power, power}});
            // search for the first col
            double t0 = getRuntime();
            while (opModeIsActive() && range00 - range.getDistance(DistanceUnit.CM) < dist2 && getRuntime() - t0 < 2) {
                pidErr = GyroPID(0, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
                telemetry.addData("range0: ", range00);
                telemetry.addData("range1: ", range.getDistance(DistanceUnit.CM));
                telemetry.addData("dist2: ", dist2);
                telemetry.update();
            }
            range11 = range00;
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            range00 = range.getDistance(DistanceUnit.CM);

            if (sumCol != colVu - 1) {
                power = 0.6;
                setMotorPower(new double[][]{{power, -power}, {-power, power}});
                // search for the first col
                t0 = getRuntime();
                while (opModeIsActive() && range.getDistance(DistanceUnit.CM) - range00 < dist3 && getRuntime() - t0 < 2) {
                    pidErr = GyroPID(0, pidErr[1]);
                    setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
                    telemetry.addData("range0: ", range00);
                    telemetry.addData("range1: ", range.getDistance(DistanceUnit.CM));
                    telemetry.addData("dist3: ", dist3);
                    telemetry.update();
                }
                setMotorPower(new double[][]{{0, 0}, {0, 0}});

            }

            telemetry.addData("range0: ", range00);
            telemetry.addData("range1: ", range.getDistance(DistanceUnit.CM));
            telemetry.addData("dist2: ", dist2);
            telemetry.update();
        }
    }

    public void colFind(ModernRoboticsI2cRangeSensor range, int direction, double power,
                        double maxTime, int color) {//0= red , 1=blue
        // get reference rangea 0=right, 1=left
        double dist = 25;
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

        pidErr[1] = 0;
        double dist2 = 4.5;
        double range00 = range.getDistance(DistanceUnit.CM);
        telemetry.addData("RANGE", range.getDistance(DistanceUnit.CM));
        telemetry.update();

        setMotorPower(new double[][]{{power, -power}, {-power, power}});
        // search for the first col
        Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                (int) (sensorColorLeft.green() * SCALE_FACTOR),
                (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                hsvValues1);
        double t0 = getRuntime();
        if (color == 0) {

            while (opModeIsActive() && hsvValues1[0] > redColorLeftSensor &&
                    Math.abs(range00 - range.getDistance(DistanceUnit.CM)) < dist2 && getRuntime() - t0 < maxTime) {
                Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                        (int) (sensorColorLeft.green() * SCALE_FACTOR),
                        (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                        hsvValues1);
                pidErr = GyroPID(0, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
                telemetry.addData("RANGE", range.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        } else if (color == 1) {

            while (opModeIsActive() && hsvValues1[0] < blueColorLeftSensor &&
                    Math.abs(range00 - range.getDistance(DistanceUnit.CM)) < dist2 && getRuntime() - t0 < maxTime) {
                Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                        (int) (sensorColorLeft.green() * SCALE_FACTOR),
                        (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                        hsvValues1);
                pidErr = GyroPID(0, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
                telemetry.addData("RANGE", range.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        if (getRuntime() - t0 > maxTime) {//only for checks
            telemetry.addLine("IN TIME ");
            telemetry.update();
//            sleep(3000);
            colTime = true;
        }
        if (range.getDistance(DistanceUnit.CM) == 0) {//only for checks
            telemetry.addLine("RANGE SENSOR NOT WARKING");
            telemetry.update();
//            sleep(3000);
        }
        telemetry.addData("RANGE", range.getDistance(DistanceUnit.CM));
        if (range00 - range.getDistance(DistanceUnit.CM) > 0) {
            telemetry.addData("Found Wall   ", range00 - range.getDistance(DistanceUnit.CM));
            telemetry.update();
//            sleep(3000);
        } else if (range00 - range.getDistance(DistanceUnit.CM) < 0) {
            telemetry.addData("Found COLUMN   ", range00 - range.getDistance(DistanceUnit.CM));
            telemetry.update();
//            sleep(3000);

        }
    }

    public void colFindDistOrColor(ModernRoboticsI2cRangeSensor range, int direction, double power,
                                   double maxTime, int color, double angle) {//0= red , 1=blue
        // get reference rangea 0=right, 1=left
//        sleep(100);
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

        pidErr[1] = 0;
        double dist2 = 4.5;
        double range00 = range.getDistance(DistanceUnit.CM);
        telemetry.addData("RANGE", range.getDistance(DistanceUnit.CM));
        telemetry.update();

        setMotorPower(new double[][]{{power, -power}, {-power, power}});
        // search for the first col
        Color.RGBToHSV((int) (colorMiddle.red() * SCALE_FACTOR),
                (int) (colorMiddle.green() * SCALE_FACTOR),
                (int) (colorMiddle.blue() * SCALE_FACTOR),
                hsvValuesMiddle);
        double t0 = getRuntime();
        if (color == 0) {
            double tGlyph = getRuntime();
            t0 = getRuntime();
            while (!touchUp.getState() && opModeIsActive() && hsvValuesMiddle[0] > redColorMiddle &&
                    Math.abs(range00 - range.getDistance(DistanceUnit.CM)) < dist2 && range.getDistance(DistanceUnit.CM) != 0 && getRuntime() - t0 < maxTime) {
                ResetHue(colorMiddle, hsvValuesMiddle);
                setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});

                pidErr = GyroPID(angle, pidErr[1]);
//                servoGlyphBring[0].setPosition(0);
//                servoGlyphBring[1].setPosition(1);

                telemetry.addData("RANGE", range.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            int d = -90;
            if (color == 1)
                d = 90;
            if (range.getDistance(DistanceUnit.CM) == 0)
                driveByEncoder(10, 1, d);
            servoGlyphBring[0].setPosition(0.5);
            servoGlyphBring[1].setPosition(0.5);
            if (!(range00 - range.getDistance(DistanceUnit.CM) > dist2 || hsvValuesMiddle[0] < redColorMiddle)) {
                tGlyph = getRuntime();
                t0 = getRuntime();
                while (!touchUp.getState() && tGlyph + 0.3 > getRuntime() && opModeIsActive() && hsvValuesMiddle[0] > redColorMiddle &&
                        Math.abs(range00 - range.getDistance(DistanceUnit.CM)) < dist2 && range.getDistance(DistanceUnit.CM) != 0 && getRuntime() - t0 < maxTime) {
                    ResetHue(colorMiddle, hsvValuesMiddle);
                    setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});

                    pidErr = GyroPID(angle, pidErr[1]);

                    telemetry.addData("RANGE", range.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }
            }

            if (range00 - range.getDistance(DistanceUnit.CM) < 0)
                negative = true;
            telemetry.addData("RANGE", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        } else if (color == 1)

        {

            t0 = getRuntime();
            while (opModeIsActive() && !touchUp.getState() && hsvValuesMiddle[0] < blueColorMiddle &&
                    Math.abs(range00 - range.getDistance(DistanceUnit.CM)) < dist2 && range.getDistance(DistanceUnit.CM) != 0 &&
                    getRuntime() - t0 < maxTime) {

                ResetHue(colorMiddle, hsvValuesMiddle);
//                servoGlyphBring[0].setPosition(0);
//                servoGlyphBring[1].setPosition(1);
                pidErr = GyroPID(angle, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
                telemetry.addData("RANGE", range.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            servoGlyphBring[0].setPosition(0.5);
            servoGlyphBring[1].setPosition(0.5);

            if (!(range00 - range.getDistance(DistanceUnit.CM) > dist2 || hsvValuesMiddle[0] > blueColorMiddle)) {
                t0 = getRuntime();

                while (opModeIsActive() && hsvValuesMiddle[0] < blueColorMiddle &&
                        Math.abs(range00 - range.getDistance(DistanceUnit.CM)) < dist2 && range.getDistance(DistanceUnit.CM) != 0 &&
                        getRuntime() - t0 < maxTime) {
                    ResetHue(colorMiddle, hsvValuesMiddle);

                    pidErr = GyroPID(angle, pidErr[1]);
                    setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
                    telemetry.addData("RANGE", range.getDistance(DistanceUnit.CM));

                }
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
            if ((range00 - range.getDistance(DistanceUnit.CM)) < 0) {
                col_ = true;
            }
            if (hsvValuesMiddle[0] > blueColorMiddle && color == 1)

            {
                telemetry.addLine("BLUE");
                telemetry.update();
            }
            if (hsvValuesMiddle[0] < redColorMiddle && color == 0)

            {
                telemetry.addLine("RED");
                telemetry.update();
            }
            if (getRuntime() - t0 > maxTime)

            {//only for checks
                telemetry.addLine("IN TIME ");
                telemetry.update();
//            sleep(3000);
                colTime = true;
            }
            if (range.getDistance(DistanceUnit.CM) == 0)

            {//only for checks
                telemetry.addLine("RANGE SENSOR NOT WARKING");
                telemetry.update();
            }
            telemetry.addData("RANGE", range.getDistance(DistanceUnit.CM));
            if (range00 - range.getDistance(DistanceUnit.CM) > 0)

            {
                telemetry.addData("Found Wall   ", range00 - range.getDistance(DistanceUnit.CM));
                telemetry.update();
//            sleep(3000);
            } else if (range00 - range.getDistance(DistanceUnit.CM) < 0)

            {
                telemetry.addData("get UP DOOR   ", range00 - range.getDistance(DistanceUnit.CM));
                telemetry.update();

                time = getRuntime();
//            setMotorPower(new double[][]{{0, 0}, {0, 0}});
//            while (opModeIsActive() && !touchUp.getState() && getRuntime() - time < 1.5) {
//                servoGlyphBring[0].setPosition(0);
//                servoGlyphBring[1].setPosition(1);
//            }
//            servoGlyphBring[0].setPosition(0.5);
//            servoGlyphBring[1].setPosition(0.5);
//        }
            }
        }

    }

    public void colFindLiftColor(int direction, double power, double maxTime, int color, double heading) {//0= red , 1=blue
        // get reference rangea 0=right, 1=left

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
        pidErr[1] = 0;
        setMotorPower(new double[][]{{power, -power}, {-power, power}});
        // search for the first col
        ResetHue(colorMiddle, hsvValuesMiddle);
        double t0 = getRuntime();
        if (color == 0) {
//            double tGlyph = getRuntime();
            while (opModeIsActive() && hsvValuesMiddle[0] > redColorMiddle && getRuntime() - t0 < maxTime) {
                ResetHue(colorMiddle, hsvValuesMiddle);
                pidErr = GyroPID(heading, pidErr[1]);
//                if (!touchUp.getState() && tGlyph + 0.3 > getRuntime()) {
//                    servoGlyphBring[0].setPosition(0);
//                    servoGlyphBring[1].setPosition(1);
//                } else {
//                    servoGlyphBring[0].setPosition(0.5);
//                    servoGlyphBring[1].setPosition(0.5);
//                }
            }

            setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});
        } else if (color == 1)

        {
            double tGlyph = getRuntime();
            while (opModeIsActive() && hsvValuesMiddle[0] < blueColorMiddle && getRuntime() - t0 < maxTime) {
                ResetHue(colorMiddle, hsvValuesMiddle);
//                if (!touchUp.getState() && tGlyph + 0.3 > getRuntime()) {
//                    servoGlyphBring[0].setPosition(0);
//                    servoGlyphBring[1].setPosition(1);
//                } else {
//                    servoGlyphBring[0].setPosition(0.5);
//                    servoGlyphBring[1].setPosition(0.5);
//                }
                pidErr = GyroPID(heading, pidErr[1]);
                setMotorPower(new double[][]{{power - pidErr[0], -power + pidErr[0]}, {-power - pidErr[0], power + pidErr[0]}});

            }
        }

        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        if (getRuntime() - t0 > maxTime)

        {//only for checks
            telemetry.addLine("IN TIME ");
            telemetry.update();
//            sleep(3000);
            colTime = true;
        }


    }

    public int GetColoumn() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ATwvQoX/////AAABmQCUHaGI6UcBuCKaYDwycSNZevdRWH+fhqsN9pcHj/QEEZiJZX6SxwENPEKL4KfPkn4IEwgmjvnrbNvh/00C7jGELQMZkToqv07vWDxA1nNvtfOGzgc15YamWbj6BkT24DtHwP4Tv53RfUolZIqSOor0GNeM9dHdVSQOB02j3t+iuXSyHcgc5vmzHBrTdYXfwISHh+je+zY99cv9rGS/edQQCSQ852ityUU5vtiYqtYkOidx47DjozTnI8UkRHENpcNnsyHCZNUA2z7Yqyt//wVdigQ5HkGlaNNo1Lhoh9koRkRNMN/ZbpclbMx20aWHvsb2PAD5YbfDub8qtMiLBGAZ+8Lsn0CDw81N/zN9ivRd";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //CameraDevice.getInstance().setField("iso","1600");
        //CameraDevice.getInstance().setField("white-balance","off");
        CameraDevice.getInstance().setFocusMode(CameraDevice.FOCUS_MODE.FOCUS_MODE_MACRO);
      //  CameraDevice.getInstance().selectVideoMode(CameraDevice.MODE.MODE_OPTIMIZE_QUALITY);


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        int col = 2;
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        //   while (!gamepad1.a) {vu

        vuMark = RelicRecoveryVuMark.UNKNOWN;
        while (!opModeIsActive() && !isStarted()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            //while (opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN && (getRuntime() - time0) < 1) {
            // sleep(nSleep);
            telemetry.addData("VuMark", "%s visible", col);

//                telemetry.update();

            //}
//            sleep(nSleep);
            telemetry.addLine("VuMark FOUND");
            telemetry.update();
            if (vuMark == RelicRecoveryVuMark.LEFT)
                col = 3;
            if (vuMark == RelicRecoveryVuMark.CENTER)
                col = 2;
            if (vuMark == RelicRecoveryVuMark.RIGHT)
                col = 1;
        }


//        sleep(nSleep);

        // relicTrackables.deactivate();
        relicTrackables.deactivate();

        // this.vuforia.
        return col;

    }


    public void knockBall(int color) {
//        double pos1 = 0.8;  //0.4 is the jwells
//        double pos2 = 0.5;  //0.35 to all sides. 0.5 is between
        // 0=red,1=blue
        telemetry.addLine("IN KnockBall");
        telemetry.update();

        if (opModeIsActive()) {


            double pos = 0.6;  // servo sensor initiallized position

            while (opModeIsActive() && pos > 0.175) { //Servo sensor goes down
                pos -= 0.007;
                servoSensor.setPosition(pos);
//                sleep(50);
                telemetry.addData("IN SERVO DOWN", pos);
                telemetry.update();
            }
            // sleep(1400);
            sleep(700);

            if (opModeIsActive() && ((sensorColor.blue() > sensorColor.red() && color == 1) ||
                    (sensorColor.blue() < sensorColor.red() && color == 0))) {
                telemetry.addData(">", "Color Checked", servoSensor.getPosition());
                telemetry.update();
                servoArm.setPosition(0.2);
                sleep(250);
            } else if (opModeIsActive() && ((sensorColor.red() > sensorColor.blue() && color == 1) ||
                    (sensorColor.red() < sensorColor.blue() && color == 0))) {
                telemetry.addData(">", "ColorNoRight", servoArm.getPosition());
                telemetry.update();
                servoArm.setPosition(0.8);
                sleep(250);
            }

            servoArm.setPosition(0.5);//servo arm goes to start posision
            pos = 0;
            while (pos < 0.8 && opModeIsActive()) {
                servoSensor.setPosition(pos);
//                sleep(nSleep);
                pos += 0.05;
//                sleep(50);
                telemetry.addData("servoSensor UP", pos);
                telemetry.update();
            }

        }
    }


}