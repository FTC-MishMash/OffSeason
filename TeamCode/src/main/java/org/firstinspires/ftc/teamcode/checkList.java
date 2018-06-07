package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static java.lang.Thread.sleep;


@TeleOp(name = "CHECK", group = "Iterative Opmode")
@Disabled
public class checkList extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor[][] drivetrainDC = new DcMotor[3][3];
   // DcMotor[] intakeDC = new DcMotor[2];
    DcMotor[] relicDC = new DcMotor[1];
    double[][] sticks = new double[4][4];
    Servo[] servosGlip = new Servo[3];
    Servo servoSensor;
    Servo servoArm;
    DistanceSensor sensorColorDistanse;
    BNO055IMU imu;
    Servo[] servosRelic = new Servo[2];
    //ModernRoboticsI2cRangeSensor range;
    ColorSensor sensorColor;
    ColorSensor sensorColorBack;
    ColorSensor sensorGlip1;
    ColorSensor sensorGlip2;
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
    int nSleep = 150;
    final double SCALE_FACTOR = 255;
  //  AnalogInput pot;
    I2cDeviceSynch pixyCam;

    double x1, y1, width1, height1, numObjects1;
    double x2, y2, width2, height2, numObjects2;
    AnalogInput pot;

    byte[] pixyData1, pixyData2;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
        //intakeDC[0].setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrainDC[1][1].setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrainDC[1][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDC[1].setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        checkList();
    }

    public void checkList() {

        boolean x = false, y = false, a = false, b = false, rangeSensor = false, rightBumper = false, leftstickbutton = false;

        telemetry.addLine("Press X");
        telemetry.update();

        if (gamepad2.x) {
            telemetry.addLine("You have startrd the CHECK LIST");
            telemetry.update();
            Sleep(1000);
            telemetry.addLine("The gliph is get IN");
            telemetry.update();
            intakeOperation(0.7);
            Sleep(2000);
            intakeOperation(0);
            telemetry.addLine("The gliph is get OUT");
            telemetry.update();
            intakeOperation(-0.7);
            Sleep(2000);
            intakeOperation(0);
            x = true;
        }
        if(gamepad2.right_stick_button){  telemetry.addData("colorDist",sensorColorDistanse.getDistance(DistanceUnit.CM));
        telemetry.update();
        }
        telemetry.addLine("Press LeftStickButton");
        telemetry.update();
        Sleep(2000);

        if (gamepad2.left_stick_button) {
            telemetry.addData("Jewel red", sensorColor.blue());
            telemetry.addData("Jewel blue", sensorColor.red());
            Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                    (int) (sensorColorBack.green() * SCALE_FACTOR),
                    (int) (sensorColorBack.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Back color", hsvValues[0]);
            Sleep(1000);

            telemetry.addLine("press Y");
            telemetry.update();
            Sleep(1000);
            if (gamepad2.y) {
                telemetry.addLine("the Door UP");
                telemetry.update();
                glyphsServoOperation(0, 1);
                telemetry.addLine("the clamps get OPEN");
                telemetry.update();
                Sleep(2000);
                glyphsServoOperation(1, 1);
                telemetry.addLine("the clamps get CLOSE");
                telemetry.update();
                Sleep(2000);
                glyphsServoOperation(1, 0);
                y = true;
            }
            telemetry.addLine("press B");
            telemetry.update();
            Sleep(1000);
            if (gamepad2.b) {
                telemetry.addLine("The robot TURN 90");
                telemetry.update();
                Sleep(2000);
                TurnGilat2(90, 1);
                telemetry.addLine("The robot TURN LEFT");
                telemetry.update();
                Sleep(1000);
                TurnGilat2(270, -1);
                b = true;

            }
            telemetry.addData("press X Game Pad 1",Pixy(1));
            telemetry.update();
            if(gamepad1.x){
                telemetry.addData("pixy",Pixy(1));
                telemetry.update();
                Sleep(5000);
            }
            telemetry.addLine("press a");
            telemetry.update();
            if (gamepad2.a) {
                telemetry.addLine("The RELIC Will be OPEN");
                telemetry.update();
                Sleep(2000);
                int relicPos = relicDC[0].getCurrentPosition();
                telemetry.addData("pos", relicPos);
                //opens the relic arm
                relicDC[0].setTargetPosition(relicPos + 50);
                relicDC[0].setPower(1);
                telemetry.addLine("RELIC will be CLODES");
                telemetry.update();
                Sleep(2000);

                relicDC[0].setTargetPosition(relicPos - 50);
                relicDC[0].setPower(1);
                telemetry.addLine("servos RELIC     1");
                telemetry.update();
                Sleep(2000);

                servosRelic[0].setPosition(1);
                servosRelic[0].setPosition(0);
                telemetry.addLine("servos RELIC     1");
                telemetry.update();
                Sleep(2000);

                servosRelic[1].setPosition(0);
                servosRelic[1].setPosition(0.5);
                a = true;
            }
            telemetry.addLine("press RIGHT BUMPER");
            telemetry.update();
            Sleep(1500);
            if (gamepad2.right_bumper) {
                telemetry.addLine("check RANGE SENSOR");
                telemetry.update();
                Sleep(1500);
                time = getRuntime();
                while (time - getRuntime() < 5) {
                    telemetry.addLine("check RANGE");
                    telemetry.addData("range", range.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }
            }


            if (!a || !b || !y || !x)

            {
                telemetry.addData("y=", y);
                telemetry.addData("x=", x);
                telemetry.addData("a=", a);
                telemetry.addData("b=", b);
                telemetry.update();
                Sleep(5000);
            }
            if (a == true && b == true && y == true && x == true && rightBumper == true)
                telemetry.addLine("You are finish the CHECK LIst");
            telemetry.update();

            Sleep(1000);

        }}
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


    void glyphsServoOperation(int index, double position) {
        servosGlip[index].setPosition(position);
        telemetry.addData("pot",pot.getVoltage());
        telemetry.update();
    }

    void intakeOperation(double power) {
        int i = 0;
        for (i = 0; i <= 1; i++)
            intakeDC[i].setPower(power);
    }

    void Sleep(int time) {
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void TurnGilat2(double angle, int direction) {
        telemetry.addLine("IN TURN:");
        telemetry.update();
        Sleep(500);
        //if(direction==-1)
        // setMotorPower(new double[][]{{-0.7, 0.7}, {-0.7, 0.7}});
        //sleep(sleeptime)
        //else
        // setMotorPower(new double[][]{{0.7, -0.7}, {0.7, -0.7}});
        //sleep(sleeptime)


        double angle0 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int degel = 1;
        if ((angle0 <= 0)) {
            angle0 += 360;
            degel = 0;
        }


        //     double target = Math.abs(angle0 - angle);
        if (degel == 0) {
            if (direction == -1)
                while ((angle0 > angle)) {
                    angle0 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    if (angle0 <= 0) {
                        angle0 += 360;
                    } else {
                        degel = 1;
                        break;
                    }
                    telemetry.addData("in the negative", angle0);
                    telemetry.update();

                    // setMotorPower(new double[][]{{-0.7, 0.7}, {-0.7, 0.7}});
                }
            else
                while ((angle0 > angle)) {
                    angle0 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    if ((angle0 <= 0)) {
                        angle0 += 360;
                    } else {
                        degel = 1;
                        break;
                    }
                    telemetry.addData("in the negative", angle0);
                    telemetry.update();

                    // setMotorPower(new double[][]{{-0.7, 0.7}, {-0.7, 0.7}});
                }
        }
        if (degel == 1) {
            //   target -= 180;

            if (direction == -1) {
                while ((angle0 < angle)) {
                    angle0 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    if ((angle0 <= 0)) {
                        angle0 += 360;
                        degel = 0;
                        break;
                    }

                    telemetry.addData("in the positive", angle0);
                    telemetry.update();

                    // setMotorPower(new double[][]{{-0.7, 0.7}, {-0.7, 0.7}});
                }
            } else
                while ((angle0 < angle)) {
                    angle0 = imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    if ((angle0 <= 0)) {
                        angle0 += 360;
                        degel = 0;
                        break;
                    }
                    telemetry.addData("in the positive", angle0);
                    telemetry.update();

                    // setMotorPower(new double[][]{{-0.7, 0.7}, {-0.7, 0.7}});
                }
        }
    }


    @Override

    public void stop() {

    }

}