package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Blue Near")
//@Disabled
public class NearBlue extends NewGlobal {


    @Override

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        servoGlyphSides.setPosition(1);

        knockBall(1);


        driveByEncoder(70, 1, 38);
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90, 0.6);
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

            DriveByDistance(45, 1, range, 90, 0);
            telemetry.addLine("more Than 55");
            telemetry.update();

        }

        intakeWithSensor(0.8);
        if (range.getDistance(DistanceUnit.CM) == 0) {
            driveByEncoder(70, 1, 0);
            telemetry.addLine("no range");
            telemetry.update();
        } else DriveByDistance(distFromWall - 3.5, 0.28, range, 90, 2);
        stopIntake();
        if (!range0) {
            driveByEncoder(20, 0.6, 0);
        }
//        servoGlyphSides.setPosition(0.1);

        driveByEncoder(30, 1, 90);

        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90, 0.3);

//        colFindDistOrColor(range, 1, 0.5, 3, 0, 90);
        colFindLiftColor(0, 0.45, 3, 1, 90);
//        if (col_) {
//            driveByEncoder(10, 1, -90);
//        }
        if (colVu == 1)
            driveByEncoder(13, 1, 90);
        else if (colVu == 2)
            driveByEncoder(4.6, 1, 90);
        else if (colVu == 3)
            driveByEncoder(26, 1, 90);

        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle+90, 0.35);
        driveByEncoder(9, 1, 180);
        scoreGlyphNear(true, 1);
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

        setMotorPower(new double[][]{{1, 1}, {1, 1}});
        servoGlyphSides.setPosition(1);
        reverseIntake(1);
        sleep(100);
        intake(0.8);
        sleep(120);

//        sleep(50);
        intakeWithSensor(1);
        if (range.getDistance(DistanceUnit.CM) > 50)
            DriveByDistance(40, 1, range, 90, 0);
        if (range.getDistance(DistanceUnit.CM) == 0)
            driveByEncoder(55, 1, 0);

//        DriveByDistance(distFromWall + 5, 0.4, range, -90, 0);
//        if (range.getDistance(DistanceUnit.CM) == 0)
//            driveByEncoder(26, 0.35, 0);

        stopIntake();
        DriveByDistance(distFromWall + 3, 0.4, range, 90, 1);


        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 90, 0.4);

        scoreGlyphNear(true, 2);
        driveByEncoder(3, 0.5, 180);
        setMotorPower(new double[][]{{0.0, 0.0}, {0.0, 0.0}});
//        }
    }
}