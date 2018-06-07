package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.app.Activity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by user on 19/02/2018.
 */
@TeleOp(name = "MM")
@Disabled public class SERVOS extends OpMode {
    Servo glyph1;
    Servo glyph2;
    //Servo glyph3;
    DigitalChannel touchUp;
    DigitalChannel touchDown;
    double pow = 0.5;
    Boolean up = false;
    Boolean down = false;
    Boolean pressY = false;


    @Override
    public void init() {
        glyph1 = hardwareMap.get(Servo.class, "1");

        //     glyph3 = hardwareMap.get(Servo.class, "servoBring3");

        //glyph2.setDirection(Servo.Direction.REVERSE);
        telemetry.addData("pos", glyph1.getPosition());
        telemetry.addData("pos2", glyph2.getPosition());
        telemetry.update();

    }

    @Override
    public void loop() {
      if(gamepad1.y){
          glyph1.setPosition(1);

      }
       else if(gamepad1.x){
            glyph1.setPosition(0);

        }
        else {
          glyph1.setPosition(0.5);

      }

    }

    private void sleep(int i) {
        try {
            Thread.sleep(i);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}