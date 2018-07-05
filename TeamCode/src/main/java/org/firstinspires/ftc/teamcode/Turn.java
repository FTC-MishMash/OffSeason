package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by user on 18/06/2018.
 */

public class Turn extends NewGlobal {
    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void Turn(double goalAngle) {
        // check the side of turn (left or right)

        boolean sideOfTurn = true;//true = turn right, false = turn left
        if (goalAngle == 0) {
            if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0) {
                //  sideOfTurn= true??;    //right
            } else if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 0) {
                //  sideOfTurn= false??;    //left
            }
        }
        if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle <= 0 && goalAngle < 0 &&
                imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < goalAngle) {
            //  sideOfTurn= true??;     //right

        } else if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle <= 0 && goalAngle < 0
                && imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > goalAngle) {
            //  sideOfTurn= false??;    //left

        }
        if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >= 0 && goalAngle > 0 &&
                imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < goalAngle)

        {
            //  sideOfTurn= true??;    //right

        } else if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >= 0 && goalAngle > 0 &&
                imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > goalAngle) {
            //  sideOfTurn= false??;    //left

        } else if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >= 0 && goalAngle < 0)

        {
            if ((imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - goalAngle) >
                    (180 - imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) + 180 + goalAngle) {
                //  sideOfTurn= true???;    //right
            } else {
//  sideOfTurn= false???;    //left
            }
        } else if (imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle <= 0 && goalAngle > 0)

        {
            if ((-imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + goalAngle) <
                    (180 + imu.getAngularOrientation(AxesReference.INTRINSIC,
                            AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 180 - goalAngle)) {
                //  sideOfTurn= true???;    //right
            } else {
                //  sideOfTurn= true???;    //right
            }

        }
        if (sideOfTurn) {
            while (goalAngle < imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) {
                //motors running
            }

        } else {
            while (goalAngle > imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) {
                //motors running
            }
        }
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
    }

}
