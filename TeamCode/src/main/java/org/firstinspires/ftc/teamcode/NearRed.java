package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Red Near")
//@Disabled
public class NearRed extends NewGlobal {


    @Override

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        servoGlyphSides.setPosition(1);

        knockBall(0);


        driveByEncoder(70, 1, 142);
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 65, 0.6);
        intake(1);

        driveByEncoder(35, 1, 180);
        getGlyphFirst(1);
        intakeWithSensor(1);
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90, 0.4);

        // setMotorPower(new double[][]{{-0.8, -0.8}, {-0.8, -0.8}});
//        crossing(0.8, -0.8);
        intakeWithSensor(0.8);

        if (range.getDistance(DistanceUnit.CM) > 45) {

            DriveByDistance(45, 1, rangeLeft, -90, 0);
            telemetry.addLine("more Than 55");
            telemetry.update();

        }

        intakeWithSensor(0.8);
        if (range.getDistance(DistanceUnit.CM) == 0) {
            driveByEncoder(70, 1, 0);
            telemetry.addLine("no range");
            telemetry.update();
        } else DriveByDistance(distFromWall +2  , 0.28, rangeLeft, -90, 2);
        stopIntake();
        if (!range0) {
            driveByEncoder(20, 0.6, 0);
        }
//        servoGlyphSides.setPosition(0.1);

        driveByEncoder(30, 1, 90);

        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90, 0.3);

//        colFindDistOrColor(range, 1, 0.5, 3, 0, 90);
        colFindLiftColor(1,0.45,3,0,90);
//        if (col_) {
//            driveByEncoder(10, 1, -90);
//        }
        if (colVu == 1)
            driveByEncoder(10, 1, 90);
        else if (colVu == 2)
            driveByEncoder(8, 1, -90);
        else if (colVu == 3)
            driveByEncoder(28, 1, -90);

        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90, 0.35);
        driveByEncoder(9, 1, 180);
        scoreGlyphNear(true, 2);
//        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90, 0.35);
//        sleep(150);
        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            intake(1);
            if (colVu == 1)
                driveByEncoder(60, 1, -160);
            else if (colVu == 3)
                driveByEncoder(60, 1, 160);
            else {
                driveByEncoder(50, 1, 180);
            }
            Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90, 0.35);

            getGlyphNearNew(-1);

        }

        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90, 0.35);
//            intakeDC[0].setPower(-1);
//            intakeDC[1].setPower(-1);
//            sleep(125);
//            intake(0.9);
//            sleep(120);
//            stopIntake();
        setMotorPower(new double[][]{{1, 1}, {1, 1}});
        servoGlyphSides.setPosition(1);
        reverseIntake(1);
        sleep(100);
        intake(0.8);
        sleep(120);

//        sleep(50);
        intakeWithSensor(1);
        if (range.getDistance(DistanceUnit.CM) > 50)
            DriveByDistance(40, 1, rangeLeft, -90, 0);
        if (range.getDistance(DistanceUnit.CM) == 0)
            driveByEncoder(55, 1, 0);

//        DriveByDistance(distFromWall + 5, 0.4, range, -90, 0);
//        if (range.getDistance(DistanceUnit.CM) == 0)
//            driveByEncoder(26, 0.35, 0);

        stopIntake();
        DriveByDistance(distFromWall +3, 0.4, rangeLeft, -90, 1);

//        driveByEncoder(20, 1, -90);

//        ResetHue(colorMiddle, hsvValuesMiddle);
//        if (hsvValuesMiddle[0] != 0) {
////                telemetry.addLine("col find COLOR");
////                telemetry.update();
//            colFindLiftColor(0, 0.5, 6, 0, 90);
//            driveByEncoder(12, 1, 90);
//            driveByEncoder(5, 1, 180);
//        } else {
//            DriveByDistance(distFromWall + 5, 0.28, range, -90, 2);
//            colFindDistOrColor(range, 1, 0.55, 4, 0, 90);
//            if (col_) {
//                driveByEncoder(15, 1, -90);
//            }
//        }
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90, 0.4);

        scoreGlyphNear(true,1);
        driveByEncoder(3, 0.5, 180);
        setMotorPower(new double[][]{{0.0, 0.0}, {0.0, 0.0}});
//        }
    }
}