package org.igutech.teleop;

public abstract class Module implements Comparable<Module> {

    private int priority;
    private String name;
    private boolean enabled;


    public Module(int priority, String name) {
        this.priority = priority;
        this.name = name;
        enabled = true;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public void setEnabled(boolean status) {
        enabled = status;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public String getName() {
        return name;
    }

    public int getPriority() {
        return priority;
    }


    public void init() {}
    public void initLoop() {}
    public void start() {}
    public void loop() {}
    public void stop() {}

    @Override
    public int compareTo(Module other) {
        return other.getPriority() - this.getPriority();
    }
}
