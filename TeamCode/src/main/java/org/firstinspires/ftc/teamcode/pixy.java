package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by Antistatical on 12/3/2017.
 */

@TeleOp(name = "Pixy", group = "TeleOp")
@Disabled
public class pixy extends OpMode {
    I2cDeviceSynch pixyCam;

    double x1, y1, width1, height1, numObjects1;
    double x2, y2, width2, height2, numObjects2;


    byte[] pixyData1, pixyData2;

    @Override
    public void init() {
        pixyCam = hardwareMap.get(I2cDeviceSynch.class, "pixy");
    }


    @Override
    public void loop() {
        pixyf(0);

    }

    public void driveWithPixy(int color, double pow, double heading) {
        double pixy = pixyf(color);
        double time0 = getRuntime();

        while (/*opModeIsActive() &&*/ pixy == 0 && getRuntime() - time0 < 1) {
            //   setMotorPower(new double[][]{{pow, pow}, {pow, pow}});
            pixy = pixyf(color);
        }
        if (pixy < 100) {
            telemetry.addData("the value is", 0xff & pixyData2[1]);
            telemetry.update();
        }
        // NewdriveRobotEncoder(10, 90, 0.2, heading);
        else if (pixy > 200) {
            telemetry.addData("the value is", 0xff & pixyData2[1]);
            telemetry.update();
            // NewdriveRobotEncoder(10, -90, 0.2, heading);

            // setMotorPower(new double[][]{{0, 0}, {0, 0}});
        }
    }

    public double pixyf(int color) {
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
                telemetry.addData("pixy in the left", 0xff & pixyData2[1]);
                telemetry.update();
            }
        }

        if (x2 > 170) {
            if (x2 > 135) {
                telemetry.addData("pixy in the right", 0xff & pixyData2[1]);
                telemetry.addData("pixy in the left", 0xff & pixyData2[1]);
                telemetry.update();
            }
        }

        if (x2 == 0) {
            telemetry.addData("pixy dont see", 0xff & pixyData2[1]);
            telemetry.update();
        }

        telemetry.addData("pixy in the right", 0xff & pixyData2[1]);
        telemetry.addData("pixy in the left", 0xff & pixyData2[1]);
        telemetry.update();
        telemetry.addData("2.1", 0xff & pixyData2[1]);
        telemetry.update();
        if (color == 0)
            return (pixyData1[1]);
        else if (color == 1)
            return pixyData2[1];
        else
            return pixyData2[1];
    }

}

