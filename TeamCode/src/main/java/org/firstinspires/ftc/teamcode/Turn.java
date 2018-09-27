package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by user on 18/06/2018.
 */
@Autonomous(name = "TurnTrial")
public class Turn extends NewGlobal {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        double angleTurn = 0;
        while (opModeIsActive())
        {
            telemetry.addData("angle: ",angleTurn);
            telemetry.update();
            if (gamepad1.dpad_up){
                angleTurn += 1;
            sleep(100);}
            if (gamepad1.dpad_down){
                angleTurn -= 1;
                sleep(100);
            }
            if (gamepad1.y){
                angleTurn *= -1;
                sleep(100);
            }
            if (gamepad1.a)
                Turn(angleTurn, 0.4);
        }
        // Turn(-20, 0.4);
    }

    public void Turn(double goalAngle, double power) {
        // check the side of turn (left or right)

        boolean sideOfTurn = true;//true = turn clockwise, false = turn counter-clockwise
        if (goalAngle == 0) {
            if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0) {
                sideOfTurn = false;    //right
            } else if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 0) {
                sideOfTurn = true;    //left
            }
        }
        if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle <= 0 && goalAngle < 0 &&
                imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < goalAngle) {
            sideOfTurn = false;     //right

        } else if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle <= 0 && goalAngle < 0
                && imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > goalAngle) {
            sideOfTurn = true;    //left

        }
        if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >= 0 && goalAngle > 0 &&
                imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < goalAngle)

        {
            sideOfTurn = false;    //right

        } else if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >= 0 && goalAngle > 0 &&
                imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > goalAngle) {
            sideOfTurn = true;    //left

        } else if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >= 0 && goalAngle < 0)

        {
            if ((imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - goalAngle) >
                    (360 - imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) + goalAngle) {
                sideOfTurn = false;    //right
            } else {
                sideOfTurn = true;    //left
            }
        } else if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle <= 0 && goalAngle > 0)

        {
            if ((-imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + goalAngle) <
                    (360 + imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - goalAngle)) {
                sideOfTurn = false;    //right
            } else {
                sideOfTurn = true;    //left
            }

        }
        if (sideOfTurn) {
            setMotorPower(new double[][]{{power, -power}, {power, -power}});
            if (goalAngle > 0 && imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0)

                while (goalAngle > imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) ;
                //motors running

            else {

                while (goalAngle < imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) {
                    telemetry.addData("angle:", imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                    telemetry.update();
                }

                //motors running

            }
        } else {
            setMotorPower(new double[][]{{-power, power}, {-power, power}});
            if (goalAngle < 0 && imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 0)
                while (goalAngle < imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) ;
                //motors running

            else
                while (goalAngle > imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) ;
            //motors running

        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }

}
