package org.igutech.utils.control;

/**
 * Created by Kevin on 7/21/2018.
 */

public interface BasicController {

    /**
     * Initialize internal controller values
     */
    void init();

    /**
     * Calculate the CV for the controller with a given PV
     * @param pv Process Variable representing current state
     * @return Control Variable
     */
    double update(double pv);

    /**
     * Update the setpoint for the controller
     * @param sp the new setpoint
     */
    void updateSetpoint(double sp);

}
