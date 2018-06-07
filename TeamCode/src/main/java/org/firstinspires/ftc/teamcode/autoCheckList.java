
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by חוסידמן on 22/02/2018.
 */
@Autonomous(name = "Auto Check")
@Disabled
public class autoCheckList extends NewGlobal {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//
//        parameters.vuforiaLicenseKey = "AVMHpAz/////AAAAGcjV1wtssUJ8qDsdO8BrsiRE9AjOOxR2alp+D4tJNIKvrBbofJ+N4UD+bTze24nU/Dc9BqPcJ4f+0ZVQfGNxy40x+4U+fUB6h7a6RotwgBbPn0TZmLtqPzRzGVGx+t1buWk36b34SV7otKsNLgvf1lnUKlffmWjrIr8vbfQEsZQf4SpIzPL6i9f4Bvki4DnHf+9OX4kZ6kS1PES5WWsx6N7WIkriiYCYEa/jBFhSfG1dlOqiUDI4QZX07PEO7rlYxi+bOaokIrccDU29zD7NvYjRAUkVCvlJNZ+w20CPHIhqspfBKv5mMwiS5VXn2JzVhjwsPxaRWyTiN8bc5nZyb/MTGLIWf/E8T4axTE2fOAwT";
//
//
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//
//        telemetry.addData(">", "Press Play to start");
//        telemetry.update();
//
//        int col = 2;
//        relicTrackables.activate();
//        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        //   while (!gamepad1.a) {vu


//
        double time0 = getRuntime();
        //while (opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN && (getRuntime() - time0) < 1) {
//            vuMark = RelicRecoveryVuMark.from(relicTemplate);
//            // sleep(nSleep);
//            telemetry.addData("VuMark", "%s visible", vuMark);



        //}
        sleep(nSleep);
        telemetry.addLine("VuMark FOUND");
        telemetry.update();
        knockBall(0);
        while (opModeIsActive()&&range.getDistance(DistanceUnit.CM)<=20&&getRuntime()-time0<7)
        {
            telemetry.addData("Range", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        Turn(20,0.6);
        NewdriveRobotEncoder(2, 0, 0.5, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        scoreGlyph(true);
        double t = getRuntime();
        setMotorPower(new double[][]{{0.5, -0.5}, {-0.5, 0.5}});
        sleep(900);
        setMotorPower(new double[][]{{0, 0}, {0, 0}});
        Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                (int) (sensorColorBack.green() * SCALE_FACTOR),
                (int) (sensorColorBack.blue() * SCALE_FACTOR),
                hsvValues);

        while (opModeIsActive()&&Double.isNaN(sensorColorDistanse.getDistance(DistanceUnit.CM))) {

            telemetry.addData("glyphRange:", sensorColorDistanse.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        while (opModeIsActive()&&hsvValues[0]>5) {
            Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),
                    (int) (sensorColorBack.green() * SCALE_FACTOR),
                    (int) (sensorColorBack.blue() * SCALE_FACTOR),
                    hsvValues);
        telemetry.addData("colorRight Red", hsvValues[0]);
        telemetry.update();
    }
        while (opModeIsActive()&&hsvValues[0]<100) {
            Color.RGBToHSV((int) (sensorColorBack.red() * SCALE_FACTOR),+
                    (int) (sensorColorBack.green() * SCALE_FACTOR),
                    (int) (sensorColorBack.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("colorRight Blue", hsvValues[0]);
            telemetry.update();
        }
}
}
