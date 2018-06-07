package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

/**
 * Created by user on 08/02/2018.
 */

@TeleOp(name = "encoder")
@Disabled public class encoder extends OpMode {
    DcMotor dcRelic3;
    Servo servo;
    double servo1 = 0.5;
    private ElapsedTime runtime = new ElapsedTime();
    double time1;

    @Override
    public void init() {

        dcRelic3 = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
        dcRelic3.setDirection(DcMotorSimple.Direction.FORWARD);
        dcRelic3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcRelic3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcRelic3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcRelic3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double time = getRuntime();

    }

    @Override
    public void start() {
        dcRelic3.setTargetPosition(0);
    }

    @Override
    public void loop() {
        dcRelic3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (gamepad2.x) {
            dcRelic3.setPower(1);
        } else {
            dcRelic3.setPower(0);
        }
        if (gamepad2.dpad_up) {
            servo1 += 0.1;
            Sleep(100);
        }
        if (gamepad2.dpad_down) {
            servo1 -= 0.1;
            Sleep(100);
        }
        servo.setPosition(servo1);
        int relicEncoder = dcRelic3.getCurrentPosition();
        if (gamepad2.left_bumper) {
            dcRelic3.setTargetPosition(relicEncoder - 50);
            dcRelic3.setPower(1);
            dcRelic3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if (gamepad2.right_bumper) {
            dcRelic3.setTargetPosition(relicEncoder + 50);
            dcRelic3.setPower(1);
            dcRelic3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.a) {
            if (dcRelic3.getCurrentPosition()<140) {
                dcRelic3.setTargetPosition(relicEncoder + 50);
                dcRelic3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dcRelic3.setPower(1);

            }
        }

        telemetry.addData("servo1", servo1);
        telemetry.update();

    }

    void Sleep(int time) {
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
