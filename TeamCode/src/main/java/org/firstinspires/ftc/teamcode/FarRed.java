package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Red Far")
//@Disabled
public class FarRed extends NewGlobal {


    @Override


    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        runtime.reset();
        double turnPower = 0.4;

        servoGlyphSides.setPosition(0);
        knockBall(0);


        driveByEncoder(62.5, 0.5, 180);
        servoGlyphSides.setPosition(0);

        //DistanceAndLift(distFromWall+10,0.4);
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 5, 0.5);


//        driveByEncoder(10, 1, 90);//to right
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower - 0.1);
        driveByEncoder(10, 0.8, 90);
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower - 0.1);

        if (rangeLeft.getDistance(DistanceUnit.CM) != 0)
            DriveByDistance(distFromWall + 4, 0.28, rangeLeft, 0, 2);//to the cryptobox
        else
            DriveByDistance(distFromWall + 4, 0.28, range, 0, 2);//to the cryptobox

        if (!range0)
            driveByEncoder(20, 0.6, 0);


        if (colVu == 1)
            driveByEncoder(32, 1, -90);
        else if (colVu == 2)
            driveByEncoder(55, 1, -90);
        else if (colVu == 3)
            driveByEncoder(70.5 , 1, -90);


        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower - 0.2);
//        DriveByDistance(distFromWall, 0.3, range, 0, 0);//to the criptobox

        driveByEncoder(5, 1, 180);
        scoreGlyphFar(false, 1);
        driveByEncoder(10, 0.5, 180);
        driveByEncoder((3 - colVu) * 18, 1, -90);
//        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 30, 0.7);
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - 25, turnPower);
        servoGlyphSides.setPosition(1);//open side holder
        intakeWithSensor(1);
        driveByEncoder(40, 1, 180);//move to the glyph pit
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - 20, turnPower);
        driveByEncoder(55, 0.9, 180);//move to the glyph pit


        if (Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {
            getGlyph(-1, 0);//intake glyph second time
        }
        intakeWithSensor(0.8);
        driveByEncoder(54, 1, 0);


        stopIntake();
        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower + 0.1);


//        intakeWithSensor(0.8);

        setMotorPower(new double[][]{{1, 1}, {1, 1}});//move to the criptobox
        reverseIntake(1);
        sleep(100);
        intake(0.8);
        sleep(120);
//        DistanceAndDownUp(40, 1);
        if (range.getDistance(DistanceUnit.CM) > 50) {
            DriveByDistance(50, 1, range, 0, 0);//drive with distance to the criptobox
        }

        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower);
//        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
//                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower - 0.15);
        if (range.getDistance(DistanceUnit.CM) != 0)
            DriveByDistance(distFromWall + 5, 0.28, range, 0, 1);//drive with distance low
        else
            DriveByDistance(distFromWall + 5, 0.28, rangeLeft, 0, 1);//drive with distance low

        if (!range0) {//if the range sensor not working
            driveByEncoder(90, 0.4, 0);//go to the wall
            driveByEncoder(25, 1, 180);//go back
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        stopIntake();

        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower);


        colFindLiftColor(0, 0.55, 2.5, 0, 0);//if range sensor not warking, do colFind only with color
        driveByEncoder(4.5, 1, -90);

        Turn(-imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, turnPower);
        scoreGlyphFar2(true, 1);//put glyph
        driveByEncoder(5.8, 0.5, 180);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});


    }
}

