

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;



@TeleOp(name = "DriveUpdated", group = "Iterative Opmode")
//@Disabled
public class Drive extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor[][] drivetrainDC = new DcMotor[3][3];
    DcMotor[] intakeDC = new DcMotor[2];
    DcMotor[] relicDC = new DcMotor[1];
    double[][] sticks = new double[4][4];
    Servo[] servosGlip = new Servo[3];
    Servo servoSensor;
    Servo servoArm;
    Servo[] servosRelic = new Servo[2];
    ModernRoboticsI2cRangeSensor range;
    ColorSensor sensorColor;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

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


        for (int i = 0; i < 1; i++) {
            // for (int i = 0; i < 2; i++) {

            for (int j = 0; j < 1; j++) {
                //     for (int j = 0; j < 2; j++) {


                drivetrainDC[i][j].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drivetrainDC[i][j].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            drivetrainDC[i][1].setDirection(DcMotorSimple.Direction.REVERSE);
        }
//        intakeDC[0].setDirection(DcMotorSimple.Direction.REVERSE);

        drivetrainDC[1][1].setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrainDC[1][1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDC[1].setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        servoArm.setPosition(0.5);
        servoSensor.setPosition(0.9);
        glyphsServoOperation(2, 0.3);
        glyphsServoOperation(1,1);
        //glyphsServoOperation(0, 0.2);
        servosGlip[2].setPosition(1);
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("servoBring",servosGlip[0].getPosition());
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    double pos2 = 0.5;
    int pos[][] = new int[2][2];
    double glyphPower = 0;
    double posGlyph = 0;

    @Override
    public void loop() {
//        telemetry.addData("y: ", gamepad1.right_stick_y);
//        telemetry.addData("x: ", gamepad1.right_stick_x);
//        telemetry.addData("motor RF", dcMotor[0][1].getPower());
//        telemetry.addData("motor RB", dcMotor[1][1].getPower());
//        telemetry.addData("motor LF", dcMotor[0][0].getPower());
//        telemetry.addData("motor LB", dcMotor[1][0].getPower());
        //servoSensor.setPosition(0);
        sticks[0][0] = gamepad1.left_stick_x;
        sticks[1][0] = gamepad1.left_stick_y;
        sticks[0][1] = gamepad1.right_stick_x;
        sticks[1][1] = gamepad1.right_stick_y;
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++) {
                pos[i][j] =  drivetrainDC[i][j].getCurrentPosition();
                telemetry.addData("pos", pos[i][j]);
            }
        telemetry.update();
        double serGain = 0;
        Range.clip(1, 0, serGain);
        double gain = 1;
        double strafeGain = 1;



        if (gamepad1.left_bumper)
            gain /= 1.8;
        else if (gamepad1.right_bumper)
            gain *= 1.8;
        if (gamepad1.b == true)//switch the direction of the drivetrain
            gain *= -1;
        double gain2 = 0.5;
        if (gamepad2.left_bumper)
            gain2 /= 2;
        else if (gamepad2.right_bumper)
            gain2 *= 1.6;

       // telemetry.addData("gain:", gain);
       // telemetry.addData("servoBring",servosGlip[0].getPosition());
      //  telemetry.update();



        tankDriveTrainSetPower(gain);

        //  intakeOperation(intakeGain);

        if (gamepad1.right_trigger != 0)//strafe to the right
            strafe(1);

        else if (gamepad1.left_trigger != 0)//strafe to the left
            strafe(-1);
        else
            tankDriveTrainSetPower(gain);

        if (gamepad2.a)// intake
            intakeOperation(-1);
        else if (gamepad2.b)// backwards intake
            intakeOperation(1);
        else
            intakeOperation(0);

        if(gamepad2.left_trigger != 0)//glyphs scoring
        {
            if (servosGlip[0].getPosition() > 0)
            {
                posGlyph -= 0.01;
                servosGlip[0].setPosition(posGlyph);
            }
        }

        if(gamepad2.right_trigger != 0)
        {
            if (servosGlip[0].getPosition() < 0.8)
            {
                posGlyph += 0.01;
                servosGlip[0].setPosition(posGlyph);
              //  telemetry.addData("servoBring",servosGlip[0].getPosition());
              //  telemetry.update();
            }
        }

        if (gamepad2.y){
            double posGlip;
            for (posGlip=servosGlip[0].getPosition();posGlip>0;posGlip-=0.01){
            servosGlip[0].setPosition(posGlip);
            }
        }
        if (gamepad2.x){
            servosGlip[0].setPosition(0.5);
        }

        if(gamepad2.left_bumper)//operates the side of the glyphs servo
            servosGlip[1].setPosition(1);
        else if(gamepad2.right_bumper)
            servosGlip[1].setPosition(0);

//        if(gamepad2.dpad_up)//operates the "finger" at the back of the glyphs
//        {
//            servosGlip[2].setPosition(1.5);
//            telemetry.addData("servoHand",servosGlip[2]);
//            telemetry.update();
//            Sleep(100);
//            //glyphsServoOperation(2, 0.3);
//        }
//        else if(gamepad2.dpad_down)
//            servosGlip[2].setPosition(0.4);
//        telemetry.addData("servoHand",servosGlip[2]);
//        telemetry.update();


        if(gamepad2.right_stick_x != 0)//opens the relic arm
            relicDC[0].setPower(0.45);
        else if(gamepad2.left_stick_x != 0)//closes the relic arm
            relicDC[0].setPower(-0.45);
        else
            relicDC[0].setPower(0);
    }

    void tankDriveTrainSetPower(double gain)
    {
        drivetrainDC[0][0].setPower(gain * (gamepad1.left_stick_y));
        drivetrainDC[1][0].setPower(gain * (gamepad1.left_stick_y));
        drivetrainDC[0][1].setPower(gain * (gamepad1.right_stick_y));
        drivetrainDC[1][1].setPower(gain * (gamepad1.right_stick_y));
    }

    void strafe(double gain)
    {
        double trigger = 0;
        if(gain>0)
            trigger = gamepad1.right_trigger;
        else if(gain<0)
            trigger = gamepad1.left_trigger;

        drivetrainDC[0][1].setPower(gain  *  trigger);
        drivetrainDC[1][1].setPower(-gain *  trigger);
        drivetrainDC[0][0].setPower(-gain *  trigger);
        drivetrainDC[1][0].setPower(gain  *  trigger);
    }

    void glyphsServoOperation(int index, double position)
    {
        servosGlip[index].setPosition(position);
    }

    void intakeOperation(double power)
    {
        int i = 0;
        for(i = 0;i<=1;i++)
            intakeDC[i].setPower(power);
    }

    void Sleep(int time)
    {
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }




//            dcMotor[0][1].setPower((-1) *gamepad1.right_trigger);
//            dcMotor[1][1].setPower( gamepad1.right_trigger);
//            dcMotor[0][0].setPower( gamepad1.right_trigger);
//            dcMotor[1][0].setPower((-1) *gamepad1.right_trigger);




//            dcMotor[0][1].setPower( gamepad1.left_trigger);
//            dcMotor[1][1].setPower((-1) *gamepad1.left_trigger);
//            dcMotor[0][0].setPower((-1) *gamepad1.left_trigger);
//            dcMotor[1][0].setPower( gamepad1.left_trigger);


    @Override

    public void stop() {
    }

}