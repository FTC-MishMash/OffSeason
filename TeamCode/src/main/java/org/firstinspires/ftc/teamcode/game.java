

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

import static java.lang.Thread.sleep;


@TeleOp(name = "checkDc", group = "Iterative Opmode")
@Disabled
public class game extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Servo servoSensor;
    Servo servoArm;
    ModernRoboticsI2cRangeSensor range;
    ColorSensor sensorColor;
    Servo servoGlyph;
    DcMotor[][] dcmotor = new DcMotor[5][4];
    BNO055IMU imu;


    // servosGlip[1] = hardwareMap.get(Servo.class, "servoSides");
    //  servosGlip[2] = hardwareMap.get(Servo.class, "servoHand");
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sRange");

        servoSensor = hardwareMap.get(Servo.class, "servoSensor");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoGlyph = hardwareMap.get(Servo.class, "servoBring");
        telemetry.addData("Status", "Initialized");
        dcmotor[0][1] = hardwareMap.get(DcMotor.class, "right1");
        dcmotor[1][1] = hardwareMap.get(DcMotor.class, "right2");
        dcmotor[0][0] = hardwareMap.get(DcMotor.class, "left1");
        dcmotor[1][0] = hardwareMap.get(DcMotor.class, "left2");
        dcmotor[0][1].setDirection(DcMotorSimple.Direction.REVERSE);
        dcmotor[1][1].setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        dcmotor[0][0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcmotor[1][0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcmotor[0][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcmotor[1][1].setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    int row = 0, col = 0;


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        dcmotor[1][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmotor[1][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcmotor[0][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmotor[0][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcmotor[1][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmotor[1][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcmotor[0][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmotor[0][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double posSensor = 0.5;
    double posArm = 0.5;
    double posGlyph = 1;
    double dcPower = 0;
    int teleFlag = 1;

    @Override
    public void loop() {

// TEST FOR DC MOTOR ENCODERS
        dcPower = 0;
        if (gamepad1.dpad_up) {
            dcPower = 0.5;
            teleFlag = 1;
        }
        if (gamepad1.dpad_down) {
            dcPower = -0.5;
            teleFlag = 1;
        }
        if (gamepad1.start) {
            dcmotor[1][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[1][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcmotor[0][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[0][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcmotor[1][1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[1][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcmotor[0][0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcmotor[0][0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            teleFlag = 1;
        }
        dcmotor[0][0].setPower(dcPower);
        dcmotor[1][0].setPower(dcPower);
        dcmotor[0][1].setPower(dcPower);
        dcmotor[1][1].setPower(dcPower);


        // TEST FOR knockball mechanism
        if (gamepad2.dpad_up)//opens the relic arm
        {
            posSensor += 0.005;
            servoSensor.setPosition(posSensor);
            teleFlag = 2;
        }
        if (gamepad2.dpad_down) {
            posSensor -= 0.005;
            servoSensor.setPosition(posSensor);
            teleFlag = 2;
        }
        if (gamepad2.dpad_right)//opens the relic arm
        {
            posArm += 0.005;
            servoArm.setPosition(posArm);
            teleFlag = 2;
        }
        if (gamepad2.dpad_left) {
            posArm -= 0.005;
            servoArm.setPosition(posArm);
            teleFlag = 2;
        }
        if (gamepad2.a) {
            posGlyph -= 0.0005;
            servoGlyph.setPosition(posGlyph);
            teleFlag = 2;
        }
        if (gamepad2.b) {
            posGlyph += 0.0005;
            servoGlyph.setPosition(posGlyph);
            teleFlag = 2;
        }
        if (posArm < 0)
            posArm = 0;
        if (posArm > 1)
            posArm = 1;
        if (posGlyph < 0)
            posGlyph = 0;
        if (posGlyph > 1)
            posGlyph = 1;
        if (posSensor < 0)
            posSensor = 0;
        if (posSensor > 1)
            posSensor = 1;

        // TELEMETRY
        // telemetry.clear();



        // TEST FOR Drive By Distance
        if (gamepad1.right_bumper) {
            dcmotor[0][0].setTargetPosition(dcmotor[0][0].getCurrentPosition()+100);
            dcmotor[0][0].setPower(1);
            dcmotor[1][0].setTargetPosition(dcmotor[1][0].getCurrentPosition()+100);
            dcmotor[1][0].setPower(1);
            dcmotor[0][1].setTargetPosition(dcmotor[0][1].getCurrentPosition()+100);
            dcmotor[0][1].setPower(1);
            dcmotor[1][1].setTargetPosition(dcmotor[1][1].getCurrentPosition()+100);
            dcmotor[1][1].setPower(1);
        }
        if (gamepad1.left_bumper) {
            dcmotor[0][0].setTargetPosition(dcmotor[0][0].getCurrentPosition()-100);
            dcmotor[0][0].setPower(1);
            dcmotor[1][0].setTargetPosition(dcmotor[1][0].getCurrentPosition()-100);
            dcmotor[1][0].setPower(1);
            dcmotor[0][1].setTargetPosition(dcmotor[0][1].getCurrentPosition()-100);
            dcmotor[0][1].setPower(1);
            dcmotor[1][1].setTargetPosition(dcmotor[1][1].getCurrentPosition()-100);
            dcmotor[1][1].setPower(1);
        }
        if (gamepad1.x) {
            Turn(90);
            teleFlag = 3;
        }
        if (gamepad1.b) {
            Turn(-90);
            teleFlag = 3;
        }
        if (teleFlag == 1) {
            telemetry.addData("dcmotor[0][0]", dcmotor[0][0].getCurrentPosition());
            telemetry.addData("dcmotor[1][0]", dcmotor[1][0].getCurrentPosition());
            telemetry.addData("dcmotor[0][1]", dcmotor[0][1].getCurrentPosition());
            telemetry.addData("dcmotor[1][1]", dcmotor[1][1].getCurrentPosition());
            teleFlag = 3;
        }
        if (teleFlag == 2) {
            telemetry.addData("posSensor", posSensor);
            telemetry.addData("posSensor", posArm);
            telemetry.addData("posGlyph", posGlyph);
        }
        if (teleFlag == 3) {
            telemetry.addData("sensorDistance", range.getDistance(DistanceUnit.CM));
            telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            teleFlag = 1;

        }
        telemetry.update();

    }

    @Override
    public void stop() {
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
            if (flag1 && angle1 > 0)
                angle1 -= 360;
            while (angle1 - target > 15) {
                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (flag1 && angle1 > 0)
                    angle1 -= 360;
            }

            setMotorPower(new double[][]{{-0.4, 0.4}, {-0.4, 0.4}});
            double t0 = getRuntime();
            while (Math.abs(angle1 - target) > 0.5 && getRuntime() - t0 < 1.2) {
                double pow = 0.4 * (angle1 - target) / 15 - 0.3;
                setMotorPower(new double[][]{{-pow, pow}, {-pow, pow}});
                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (flag1 && angle1 > 0)
                    angle1 -= 360;
                telemetry.addLine("here1");
                telemetry.update();
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
        }
        if (angle > 0) {
            if (target > 180)
                flag1 = true;
            setMotorPower(new double[][]{{+0.7, -0.7}, {+0.7, -0.7}});
            double angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if (flag1 && angle1 < 0)
                angle1 += 360;
            while (angle1 < target - 15) {

                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (flag1 && angle1 < 0)
                    angle1 += 360;
            }
            setMotorPower(new double[][]{{+0.4, -0.4}, {+0.4, -0.4}});
            double t0 = getRuntime();

            while (Math.abs(angle1 - target) > 0.5 && getRuntime() - t0 < 1.2) {
                double pow = 0.4 * (angle1 - target) / 15 + 0.3;
                setMotorPower(new double[][]{{pow, -pow}, {pow, -pow}});
                angle1 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                telemetry.addLine("here2");
                telemetry.update();
                if (flag1 && angle1 < 0)
                    angle1 += 360;
            }
            setMotorPower(new double[][]{{0, 0}, {0, 0}});
        }
    }

    public void setMotorPower(double[][] power) {
        for (int row = 0; row < 2; row++)
            for (int col = 0; col < 2; col++)
                dcmotor[row][col].setPower(power[row][col]);
    }

    public void newTurn(double angle) {
        double newAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + angle;
        double power = 0;
        double pidErr[] = {0, angle};
        setMotorPower(new double[][]{{power - pidErr[0], power + pidErr[0]}, {power - pidErr[0], power + pidErr[0]}});

        while (Math.abs(pidErr[1]) > 2) {
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

}
