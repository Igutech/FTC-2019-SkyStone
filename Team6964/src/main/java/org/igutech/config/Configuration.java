package org.igutech.config;

import java.io.Serializable;
import java.util.HashMap;
import java.util.Map;

public class Configuration implements Serializable {

    private static final long serialVersionUID = -5094151129854678203L;

    private Map<String, Sheet> config;

    private String version;

    /**
     * Create a new configuration
     * @param version Version string of this configuration
     */
    public Configuration(String version) {
        this.version = version;
        config = new HashMap<>();
    }

    /**
     * Get a sheet out of the configuration
     * @param name Name of the sheet
     * @return
     */
    public Sheet getSheet(String name) {
        return config.get(name);
    }

    /**
     * Add a sheet to the configuration
     * @param name Name of the sheet
     * @param sheet Sheet object to add
     */
    public void addSheet(String name, Sheet sheet) {
        config.put(name, sheet);
    }

    /**
     * Get version string of the configuration
     * @return
     */
    public String getVersion() {
        return version;
    }

}
