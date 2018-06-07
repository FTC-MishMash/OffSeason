package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "autoTESTS")
//@Disabled
public class autoTESTS extends NewGlobal {


    @Override

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        nSleep = 500;
        double d = 0;
        double posSensor = 0.5;
        double posArm = 0.5;
        dcRelic3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcRelic3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcRelic3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double dist = 100;
        double[][] sticks = new double[4][4];
        int tAngle = 90;
        double tPower = 0.5;
        waitForStart();


        while (opModeIsActive() && !gamepad1.start) {
            if (gamepad1.right_trigger != 0)
                servoGlyphSides.setPosition(0);
            if (gamepad1.left_trigger != 0)
                servoGlyphSides.setPosition(1);
            if (gamepad1.right_stick_button)
                servoGlyphSides.setPosition(0.5);
if(gamepad1.x)
    setMotorPower(new double[][]{{0.5,0.5},{0.5,0.5}});
else setMotorPower(new double[][]{{0,0},{0,0}});
// TEST FOR knockball mechanism
            if (gamepad2.dpad_up)//opens the relic arm
            {
                posSensor += 0.005;
                servoSensor.setPosition(posSensor);

            }
            if (gamepad2.dpad_down) {
                posSensor -= 0.005;
                servoSensor.setPosition(posSensor);

            }
            if (gamepad2.dpad_right)//opens the relic arm
            {
                posArm += 0.005;
                servoGlyphSides.setPosition(posArm);
            }
            if (gamepad2.dpad_left) {
                resetZAxis();
                Turn(25, 1);
            }

//            resetZAxis();


//            if (gamepad2.left_bumper) {
//                dcRelic3.setTargetPosition(dcRelic3.getCurrentPosition() - 1);
//                dcRelic3.setPower(0.5);
//                dcRelic3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            if (gamepad2.right_bumper) {
//                dcRelic3.setTargetPosition(dcRelic3.getCurrentPosition() + 1);
//                dcRelic3.setPower(0.5);
//                dcRelic3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
////            if (gamepad1.left_trigger!=0){
////                servoSensor.setPosition(0.1);
////                sleep(1000);
////                servoArm.setPosition(0);
////                sleep(1000);
////                servoArm.setPosition(1);
////                sleep(1000);
//            int color = 1;
//            if (opModeIsActive() && ((sensorColor.blue() > sensorColor.red() && color == 1) ||
//                    (sensorColor.blue() < sensorColor.red() && color == 0))) {
//                telemetry.addData(">", "Color Checked", servoSensor.getPosition());
//                telemetry.update();
//                servoArm.setPosition(0);
//                sleep(250);
//            } else if (opModeIsActive() && ((sensorColor.red() > sensorColor.blue() && color == 1) ||
//                    (sensorColor.red() < sensorColor.blue() && color == 0))) {
//                telemetry.addData(">", "ColorNoRight", servoArm.getPosition());
//                telemetry.update();
//                servoArm.setPosition(1);
//                sleep(250);
//            }
//            servoArm.setPosition(0.3);
//            sleep(1000);
//            servoSensor.setPosition(0.9);
//            sleep(1000);
//        }

//            telemetry.addData("angle: ", imu.getAngularOrientation(AxesReference.INTRINSIC,
//                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//            telemetry.addData("tAngle: ", tAngle);
//            telemetry.addData("tPower: ", tPower);
//            telemetry.addData("SENSOR RANGE",range.getDistance(DistanceUnit.CM));
//            telemetry.addData("sensor GLYPH",(sensorColorDistanse.getDistance(DistanceUnit.CM)));
            ResetHue(sensorColorBack, hsvValues);
            ResetHue(sensorColorLeft, hsvValues1);
            ResetHue(colorMiddle, hsvValuesMiddle);

            telemetry.addData("sensor Color Right",hsvValues[0]);
            telemetry.addData("sensor Color Left",hsvValues1[0]);
        telemetry.addData("blue", sensorColor.blue());
            telemetry.addData("NAN   ", Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)));
//     telemetry.addData("magnet", magnet.getState());
            telemetry.addData("sensor Color Middle",hsvValuesMiddle[0]);
         telemetry.addData("Glyph Distance:",sensorColorDistanse.getDistance(DistanceUnit.CM));
            telemetry.addData("Range:",range.getDistance(DistanceUnit.CM));
            telemetry.addData("RangeLeft:",rangeLeft.getDistance(DistanceUnit.CM));
            telemetry.update();
        }}
    // colDrive(range,-1,0.3,2);
//            if (gamepad1.x)
//            {
//                colDrive(range, 0, 0.5 , 2);
//            }
////        knockBall(1);
////            ResetHue(sensorColorBack, hsvValues);
////            telemetry.addData("hue", hsvValues[0]);
////            telemetry.update();
////            telemetry.addLine("getDownWithColor");
////            telemetry.update();
//            if (gamepad1.y) {
//                getDownWithColor(0.28, 1);
//                sleep(nSleep);
//            }
////        sleep(500);
////        NewdriveRobotEncoder(10, 180, 0.25, 90);
//
//
//            if (gamepad1.b) {
//                Turn(75);
//                telemetry.addLine("first turn");
//                telemetry.update();
//                sleep(nSleep);
//            }
//            if (gamepad1.a) {
//                NewdriveRobotEncoder(10, 180, 0.4, 90);
//                //driveByColor(1, 0, 0.5);
//                sleep(nSleep);
//                //NewdriveRobotEncoder(10, 180, 0.25, 0);
//            }
//            if (gamepad1.right_stick_button) {
//                telemetry.addLine("second Turn");
//
//
//                Turn(75);
//
//
////        telemetry.addLine("Encoder inside");
////        telemetry.update();
//                // NewdriveRobotEncoder(10, 180, 0.4, 180);
//                telemetry.addLine("Turn Vufuria");
//                telemetry.update();
//            }
////        sleep(2000);
//            if (gamepad1.left_stick_button) {
//                if (colVu == 1) {
//                    telemetry.addLine("Turn Vufuria right");
//                    telemetry.update();
//                    Turn(-25);
//                }
//
//                //if col Vu === 2 the robot dont need to turn
//                if (colVu == 3) {
//                    telemetry.addLine("Turn Vufuria left");
//                    telemetry.update();
//                    sleep(500);
//                    Turn(75);
//                }
//            }
//            sleep(nSleep);
//            if (gamepad1.dpad_down) {
//                NewdriveRobotEncoder(10, 180, 0.5, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//inside
//                // NewdriveRobotEncoder(8, 180, 0.4, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                // AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//                intakeDC[0].setPower(1);
//                intakeDC[1].setPower(-1);
//                sleep(400);
//            }
////        setMotorPower(new double[][]{{0.7, 0.7}, {0.7, 0.7}});
////        sleep(50);
////        setMotorPower(new double[][]{{-0.7, -0.7}, {-0.7, -0.7}});
////        sleep(300);
////        setMotorPower(new double[][]{{0.7,0.7},{0.7,0.7}});
//
//            if (gamepad1.dpad_left) {
//                NewdriveRobotEncoder(5, 180, -0.7, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//outside
//                NewdriveRobotEncoder(10, 180, 0.6, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//inside
//                NewdriveRobotEncoder(15, 180, -0.6, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//outside
//                sleep(nSleep);
//            }
//            if (gamepad1.dpad_up) {
//                if (colVu == 2) {
//                    //Turn(10);
//                    NewdriveRobotEncoder(15, 180, 0.7, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//inside
////            sleep(nSleep);
////            NewdriveRobotEncoder(5, 180, -0.7,  imu.getAngularOrientation(AxesReference.INTRINSIC,
////                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//outside
//                    //Turn(-10);
//                }
//
//                if (colVu == 1) {
//                    // Turn(20);
////            NewdriveRobotEncoder(5, 180, -0.7,  imu.getAngularOrientation(AxesReference.INTRINSIC,
////                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//outside
//                    NewdriveRobotEncoder(5, 180, 0.7, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//inside
//                }
//                //if col Vu === 2 the robot dont need to turn
//                if (colVu == 3) {
////            NewdriveRobotEncoder(5, 180, -0.7,  imu.getAngularOrientation(AxesReference.INTRINSIC,
////                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//outside
//                    intakeDC[0].setPower(0);
//                    intakeDC[1].setPower(0);
//                    NewdriveRobotEncoder(5, 180, 0.7, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//inside
//                }
//                sleep(nSleep);
//
//            }
//            if (gamepad1.dpad_right) {
//                NewdriveRobotEncoder(10, 180, -0.5, imu.getAngularOrientation(AxesReference.INTRINSIC,
//                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);//outside
//
////            NewdriveRobotEncoder(10, 180, -0.5, imu.getAngularOrientation(AxesReference.INTRINSIC,
////                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
////            NewdriveRobotEncoder(25, 180, 0.4, imu.getAngularOrientation(AxesReference.INTRINSIC,
////                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
////            NewdriveRobotEncoder(20, 180, -0.6, imu.getAngularOrientation(AxesReference.INTRINSIC,
////                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//
////        setMotorPower(new double[][]{{0.7, 0.7}, {0.7, 0.7}});
////        sleep(200);
////        setMotorPower(new double[][]{{-0.7, -0.7}, {-0.7, -0.7}});
////        sleep(600);
////        setMotorPower(new double[][]{{0.7, 0.7}, {0.7, 0.7}});
////        sleep(300);
////        setMotorPower(new double[][]{{0, 0}, {0, 0}});
//                intakeDC[0].setPower(0);
//                intakeDC[1].setPower(0);
//                sleep(nSleep);
//
//            }
////            setMotorPower(new double[][]{{-1, 1}, {1, -1}});
////            sleep(900);
////            setMotorPower(new double[][]{{0, 0}, {0, 0}});
////            Turn(180);
////            driveByColor(bColor, -20, -0.35);
////            getMoreGlyph1(-0.5, bColor, -20);
////            DriveByDistance(30, 0.6, range, -20);
////            if (!range0) {
////                NewdriveRobotEncoder(50, 0, 0.55, -20);
////            }
////            NewdriveRobotEncoder(5, 0, -0.55, 180);

    void tankDriveTrainSetPower(double gain) {
        dcmotor[0][0].setPower(gain * (gamepad1.left_stick_y));
        dcmotor[1][0].setPower(gain * (gamepad1.left_stick_y));
        dcmotor[0][1].setPower(gain * (gamepad1.right_stick_y));
        dcmotor[1][1].setPower(gain * (gamepad1.right_stick_y));
    }
    public void getGlyph1(double power) { //Drives to the Glyph pit and gathers until 2 glyphs
        //0= red,1= blue
        double pidErr[] = {0, 0};
        boolean cros = false;
        double volt = readVoltage();
        servoGlyphSides.setPosition(1); //open glyphs holder
        intakeWithSensor(1);
        double time = getRuntime();
        ResetHue(sensorColorBack, hsvValues);
        telemetry.addData("sensor Range rev", sensorColorDistanse.getDistance(DistanceUnit.CM));
        telemetry.update();
//        sleep(500);
        while (opModeIsActive() && volt > voltageMin && Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)))//In loop until there is glyph pn the robot
        {
            volt = readVoltage();
//            pidErr = GyroPID(90, pidErr[1]);
//            setMotorPower(new double[][]{{power, power}, {power, power}});
            //setMotorPower(new double[][]{{power, power}, {power, power}});
            telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
            telemetry.update();
            //  sleep(500);
            // drive to
        }
        stopIntake();
        volt = readVoltage();
        if (volt < voltageMin) {
//            setMotorPower(new double[][]{{-power * 0.4, -power * 0.4}, {-power * 0.4, -power * 0.4}});
            if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
                intakeWithSensor(1);
            } else stopIntake();
            intakeWithSensor(1);
            Turn(10, 1);
            Turn(-10, 1);
            stopIntake();
            crossing(1, -0.5);
            telemetry.addLine("voltage is LOW");
            telemetry.addData("voltage", volt);
            telemetry.update();
            cros = true;
        }
        intakeWithSensor(1);
        telemetry.addLine("Finished loop1");
        telemetry.update();

        Turn(-12, 0.8);
        volt = readVoltage();
        time = getRuntime();
        intakeWithSensor(1);
        while (opModeIsActive() && volt > voltageMin && Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM)))//In loop until there is glyph pn the robot
        {
            volt = readVoltage();
//            intakeWithSensor(1);
            if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
                intakeWithSensor(1);
            } else stopIntake();
//            pidErr = GyroPID(90, pidErr[1]);
//            setMotorPower(new double[][]{{power, power}, {power, power}});
            //setMotorPower(new double[][]{{power, power}, {power, power}});
            telemetry.addData("sensor Range rev in the loop", sensorColorDistanse.getDistance(DistanceUnit.CM));
            telemetry.update();
            //  sleep(500);
            // drive to
        }
        stopIntake();
        intakeWithSensor(1);
        if (volt < voltageMin) {
//            setMotorPower(new double[][]{{-power * 0.4, -power * 0.4}, {-power * 0.4, -power * 0.4}});
            crossing(1, -0.7);
            telemetry.addLine("voltage is LOW");
            telemetry.addData("voltage", volt);
            telemetry.update();
            cros = true;
        }
//        intakeWithSensor(1);
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            intakeWithSensor(1);
        } else stopIntake();
        Turn(15, 0.8);


        telemetry.addLine("Finished loop2");
        telemetry.update();


        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        stopIntake();

    }
}


