package org.igutech.autonomous.util;

import org.igutech.config.ConfigManager;
import org.igutech.config.Sheet;

import java.util.HashMap;
import java.util.Map;

public class AutoConfigUtil {

    private Map<String, String> values;
    private Map<String, String> units;
    private Sheet parameters;

    public AutoConfigUtil(AutoUtilManager manager, String name) {
        //TODO: CHECK FOR NULL POINTER
        parameters = ConfigManager.getConfig().getSheet("auton_" + name);
        values = new HashMap<>();
        units = new HashMap<>();

        int i = 2;
        while (parameters.getContents().get("A").containsKey(i)) {
            values.put(parameters.getContents().get("A").get(i), parameters.getContents().get("B").get(i));
            if (parameters.getContents().get("C").containsKey(i))
                values.put(parameters.getContents().get("A").get(i), parameters.getContents().get("C").get(i));
        }
    }

    public String getValue(String key) {
        return values.get(key);
    }

    public String getUnits(String key) {
        return units.get(key);
    }

}
