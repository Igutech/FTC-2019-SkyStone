package org.igutech.utils.control;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PControllers extends PIDFControllers {

    public PControllers(double p) {
        super(new double[]{p, 0, 0, 0});
    }

    public PControllers(double p, double sp, double pv, double period) {
        super(new double[]{p, 0, 0, 0}, sp, pv, period);
    }

    public void pControl(DcMotor affected, double sp, double pv) {
        affected.setPower(super.calculate(pv, sp));
    }

}