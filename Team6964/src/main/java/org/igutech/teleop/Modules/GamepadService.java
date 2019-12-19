package org.igutech.teleop.Modules;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.igutech.config.ConfigManager;
import org.igutech.teleop.Service;

public class GamepadService extends Service {

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public GamepadService(Gamepad gamepad1, Gamepad gamepad2) {
        super("GamepadService");
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Retrieve an output of a digital button on a gamepad
     * @param gamepad ID of the gamepad (1 or 2).
     * @param id string name of gamepad parameter ("dpad_up", "left_bumper")
     * @throws IllegalArgumentException if the id or gamepad number is invalid
     * @return
     */
    public boolean getDigital(int gamepad, String id) {
        Gamepad target;
        if (gamepad == 1) {
            target = gamepad1;
        } else if (gamepad == 2) {
            target = gamepad2;
        } else {
            throw new IllegalArgumentException("Illegal Argument: gamepad must be 1 or 2");
        }

        id = id.toLowerCase();
        switch(id) {
            case "dpad_up":
                return target.dpad_up;
            case "dpad_down":
                return target.dpad_down;
            case "dpad_left":
                return target.dpad_left;
            case "dpad_right":
                return target.dpad_right;
            case "a":
                return target.a;
            case "b":
                return target.b;
            case "x":
                return target.x;
            case "y":
                return target.y;
            case "guide":
                return target.guide;
            case "start":
                return target.start;
            case "back":
                return target.back;
            case "left_bumper":
                return target.left_bumper;
            case "right_bumper":
                return target.right_bumper;
            case "right_stick_button":
                return target.right_stick_button;
            case "left_stick_button":
                return target.left_stick_button;
        }
        throw new IllegalArgumentException("Illegal Argument: id");
    }

    /**
     * Retrieve an output of an analog control on a gamepad
     * @param gamepad ID of the gamepad (1 or 2).
     * @param id string name of gamepad parameter ("left_stick_x", "right_trigger")
     * @throws IllegalArgumentException if the id or gamepad number is invalid
     * @return
     */
    public double getAnalog(int gamepad, String id) {
        Gamepad target;
        if (gamepad == 1) {
            target = gamepad1;
        } else if (gamepad == 2) {
            target = gamepad2;
        } else {
            throw new IllegalArgumentException("Illegal Argument: gamepad must be 1 or 2");
        }

        id = id.toLowerCase();
        switch (id) {
            case "left_stick_x":
                return target.left_stick_x;
            case "left_stick_y":
                return target.left_stick_y;
            case "right_stick_x":
                return target.right_stick_x;
            case "right_stick_y":
                return target.right_stick_y;
            case "left_trigger":
                return target.left_trigger;
            case "right_trigger":
                return target.right_trigger;
        }
        throw new IllegalArgumentException("Illegal Argument: id");
    }

    public Object getValueFromConfig(MappingValues value) {
        String control = ConfigManager.getConfig()
                .getSheet(value.getSheet())
                .get(value.getCol(), value.getRow());

        if (value.getType().equals(Type.ANALOG)) {
            double analogValue = getAnalog(value.getGamepad(), control);
            if (value.reversable && ConfigManager.getConfig()
                    .getSheet(value.getSheet())
                    .get(value.getCol()+1, value.getRow()).equalsIgnoreCase("TRUE")) {
                return -analogValue;
            } else {
                return analogValue;
            }
        } else {
            return getDigital(value.getGamepad(), control);
        }
    }

    public enum Type {
        ANALOG,
        DIGITAL;
    }

    public enum MappingValues {

        FORWARDS_BACKWARDS("gamepad1_mapping", "B", 2, true, Type.ANALOG, 1),
        STRAFE("gamepad1_mapping", "B", 3, true, Type.ANALOG, 1),
        YAW("gamepad1_mapping", "B", 4, true, Type.ANALOG, 1),
        SLOWMO("gamepad1_mapping", "B", 5, true, Type.ANALOG, 1);

        private String sheet;
        private String col;
        private int row;
        private boolean reversable;
        private Type type;
        private int gamepad;

        private MappingValues(String sheet, String col, int row, boolean reversable, Type type, int gamepad) {
            this.sheet = sheet;
            this.col = col;
            this.row = row;
            this.reversable = reversable;
            this.type = type;
            this.gamepad = gamepad;
        }

        public String getSheet() {
            return sheet;
        }

        public String getCol() {
            return col;
        }

        public int getRow() {
            return row;
        }

        public boolean isReversable() {
            return reversable;
        }

        public Type getType() {
            return type;
        }

        public int getGamepad() {
            return gamepad;
        }
    }
}
