package org.igutech.teleop;

public abstract class Service extends Module {
    public Service(String name) {
        super(Integer.MAX_VALUE, name);
    }

    /**
     * Need to disable this function, services cannot be disabled!
     * @param enabled
     */
    @Override
    public void setEnabled(boolean enabled) {
        super.setEnabled(true);
    }

    /**
     * Need to disable this function, services cannot be disabled!
     */
    @Override
    public void disable() {
        return;
    }
}
