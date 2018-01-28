

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Thread.sleep;


@TeleOp(name = "fod", group = "Iterative Opmode")
@Disabled
public class Drive_fod extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor[][] drivetrainDC = new DcMotor[3][3];
    DcMotor[] intakeDC = new DcMotor[2];
    DcMotor[] relicDC = new DcMotor[1];
    double[][] sticks = new double[4][4];
    Servo[] servosGlip = new Servo[3];
    Servo servoSensor;
    Servo servoArm;
    Servo[] servosRelic = new Servo[2];
    ModernRoboticsI2cRangeSensor range;
    ColorSensor sensorColor;
    BNO055IMU imu;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        drivetrainDC[0][1] = hardwareMap.get(DcMotor.class, "right1");
        drivetrainDC[1][1] = hardwareMap.get(DcMotor.class, "right2");
        drivetrainDC[0][0] = hardwareMap.get(DcMotor.class, "left1");
        drivetrainDC[1][0] = hardwareMap.get(DcMotor.class, "left2");

        intakeDC[0] = hardwareMap.get(DcMotor.class, "g1");
        intakeDC[1] = hardwareMap.get(DcMotor.class, "g2");
        relicDC[0] = hardwareMap.get(DcMotor.class, "dcRelic");
        servoSensor = hardwareMap.get(Servo.class, "servoSensor");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servosGlip[0] = hardwareMap.get(Servo.class, "servoBring");
        servosGlip[1] = hardwareMap.get(Servo.class, "servoSides");
        servosGlip[2] = hardwareMap.get(Servo.class, "servoHand");
        servosRelic[0] = hardwareMap.get(Servo.class, "relicServo1");
        servosRelic[1] = hardwareMap.get(Servo.class, "relicServo2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        for (int i = 0; i < 1; i++) {
            // for (int i = 0; i < 2; i++) {

            for (int j = 0; j < 1; j++) {
                //     for (int j = 0; j < 2; j++) {


                drivetrainDC[i][j].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drivetrainDC[i][j].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            drivetrainDC[i][1].setDirection(DcMotorSimple.Direction.REVERSE);
        }
//        intakeDC[0].setDirection(DcMotorSimple.Direction.REVERSE);

        drivetrainDC[1][1].setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrainDC[1][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDC[1].setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        servoArm.setPosition(0.5);
        servoSensor.setPosition(0.9);

        //glyphsServoOperation(0, 0.2);
        servosGlip[2].setPosition(1);
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("servoBring",servosGlip[0].getPosition());
        telemetry.update();
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    double pos2 = 0.5;
    int pos[][] = new int[2][2];
    double glyphPower = 0;
    double posGlyph = 0;
double head=0;
    @Override
    public void loop() {
//        telemetry.addData("y: ", gamepad1.right_stick_y);
//        telemetry.addData("x: ", gamepad1.right_stick_x);
//        telemetry.addData("motor RF", dcMotor[0][1].getPower());
//        telemetry.addData("motor RB", dcMotor[1][1].getPower());
//        telemetry.addData("motor LF", dcMotor[0][0].getPower());
//        telemetry.addData("motor LB", dcMotor[1][0].getPower());
        //servoSensor.setPosition(0);

head =  imu.getAngularOrientation(AxesReference.INTRINSIC,
        AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
telemetry.addData("heading",head);
telemetry.update();
        double gain = 1;
       // double newx1 = -gamepad1.right_stick_x*Math.cos(head)-gamepad1.right_stick_y*Math.sin(head);
       // double newy1 = -gamepad1.right_stick_x*Math.sin(head)+gamepad1.right_stick_y*Math.cos(head);
        double newx2 = gamepad1.right_stick_x*Math.cos(head)-gamepad1.right_stick_y*Math.sin(head);
        double newy2 = gamepad1.right_stick_x*Math.sin(head)+gamepad1.right_stick_y*Math.cos(head);
        double newx3 = gamepad1.left_stick_x*Math.cos(head)-gamepad1.left_stick_y*Math.sin(head);
        double newy3 = gamepad1.left_stick_x*Math.sin(head)+gamepad1.left_stick_y*Math.cos(head);
       // double newx4 = -gamepad1.left_stick_x*Math.cos(head)-gamepad1.right_stick_y*Math.sin(head);
       // double newy4 = -gamepad1.left_stick_x*Math.sin(head)+gamepad1.right_stick_y*Math.cos(head);

        drivetrainDC[0][1].setPower(gain * (-newx2 + newy2));
        drivetrainDC[1][1].setPower(gain * (newx2 + newy2));
        drivetrainDC[0][0].setPower(gain * (newx3+ newy3));
        drivetrainDC[1][0].setPower(gain * (-newx3 + newy3));
    }

}