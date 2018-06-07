package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerParamsState;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerPositionParams;

@TeleOp(name="voltage trial", group="Iterative Opmode")
@Disabled
public class voltageTrial extends OpMode
{
   // ExpansionHubMotorControllerParamsState vs;

    @Override
    public void init() {
       // vs = hardwareMap.get(ExpansionHubMotorControllerParamsState.class, "vs");
    }

    @Override
    public void loop() {
//        telemetry.addData("voltage",vs.p);
//        telemetry.addData("voltage",vs.d);
//        telemetry.addData("voltage",vs.i);
//        telemetry.update();
        double k= readVoltage();
        telemetry.addData("voltage: ", k);
        telemetry.update();

    }
    public double readVoltage()
    {
        double result = Double.POSITIVE_INFINITY;
        int i = 0;
        for(VoltageSensor sensor : hardwareMap.voltageSensor) {
            ++i;
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }

            telemetry.addData("voltage: ", result);
            telemetry.update();
        }
  //      telemetry.addData("voltage: ", result);
        telemetry.update();
        return result;

    }


}