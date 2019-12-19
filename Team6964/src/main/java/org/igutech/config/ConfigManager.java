package org.igutech.config;
import android.os.Environment;
import android.util.Log;
import android.widget.TextView;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.URL;

public class ConfigManager {

    public static final File ROOT_DIR = Environment.getExternalStorageDirectory();
    public static final File CONFIG_DIR = new File(ROOT_DIR + "/IGUTECH");
    public static final String TAG = "SheetConfigManager";

    public static final String CONFIG_URL = "https://docs.google.com/spreadsheets/d/1SUuYu5f-gJZFsSfsegTZOnSyWrwdRu5_xfmYYZ3f38s/gviz/tq?tqx=out:csv&sheet=";

    public static Configuration currentConfig;

    private static TextView textStatus;

    public static void setText(TextView status) {
        textStatus = status;
    }

    /**
     * Set text display on robot controller
     * @param content
     */
    private static void setContent(final String content) {
        if (textStatus == null) return;
        AppUtil.getInstance().getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                textStatus.setText(content);
            }
        });
    }

    /**
     * Initialize configuration manager
     * Loads and ensures configuration directory exists,
     * Finds the highest version configuration file and loads it
     */
    public static void initialize() {

        try {
            setContent("Loading config data...");

            File configDir = CONFIG_DIR;

            boolean createdDir = true;
            if (!configDir.exists()) {
                createdDir = configDir.mkdirs();
            }

            if (!createdDir) {
                Log.e(TAG, "Can't create sheet configuration file directory!");
            }

            File[] allFiles = configDir.listFiles(new FilenameFilter() {
                @Override
                public boolean accept(File file, String s) {
                    //Log.d(TAG, "Discovered file: " + s);
                    return s.toLowerCase().endsWith(".iconf");
                }
            });

            if (allFiles.length == 0) {
                setContent("No config!");
                return;
            }

            File highestVersion = null;
            long highestWeight = 0;
            for (File file : allFiles) {
                if (highestVersion == null) {
                    highestVersion = file;
                    continue;
                }

                if (calculateVersionWeight(file.getName()) > highestWeight) {
                    highestWeight = calculateVersionWeight(file.getName());
                    highestVersion = file;
                }
            }


            Configuration config = (Configuration) new ObjectInputStream(new FileInputStream(highestVersion)).readObject();

            setContent("Loaded: " + config.getVersion());

            currentConfig = config;
        } catch (Exception e) {
            setContent("Error reading file!");
            Log.e(TAG, "Error reading file!", e);
        }
    }

    /**
     *
     * @param filename
     * @return
     */
    private static long calculateVersionWeight(String filename) {
        String[] ids = filename.split("\\.");
        long weight = 0;
        weight += Integer.parseInt(ids[0]) * 1000000;
        weight += Integer.parseInt(ids[1]) * 1000;
        weight += Integer.parseInt(ids[2]) * 1;
        return weight;
    }

    public static void downloadConfig() {
        new Thread(() -> {
            try {
                AppUtil.getInstance().showToast(UILocation.BOTH, AppUtil.getInstance().getActivity(), "Downloading Config...");
                setContent("Downloading...");

                Sheet info = new Sheet(binToString(getSheet("info")));
                //Log.d(TAG, "Sheet information: " + info.getContents());
                Sheet gamepad1Mapping = new Sheet(binToString(getSheet("gamepad1_mapping")));
                Sheet gamepad2Mapping = new Sheet(binToString(getSheet("gamepad2_mapping")));
                Sheet autonMain = new Sheet(binToString(getSheet("auton_main")));

                Configuration configuration = new Configuration(info.get("D", 3));
                configuration.addSheet("info", info);
                configuration.addSheet("gamepad1_mapping", gamepad1Mapping);
                configuration.addSheet("gamepad2_mapping", gamepad2Mapping);
                configuration.addSheet("auton_main", autonMain);

                File target = new File(CONFIG_DIR + "/" + configuration.getVersion() + ".iconf");
                //Log.d(TAG, "Saving to: " + target.getPath());
                if (target.createNewFile()) {
                    //Log.d(TAG, "Created new file " + target.getPath());
                } else {
                    //Log.d(TAG, "Failed to create file " + target.getPath());
                }

                new ObjectOutputStream(
                        new FileOutputStream(target, false)
                ).writeObject(configuration);
                AppUtil.getInstance().showToast(UILocation.BOTH, AppUtil.getInstance().getActivity(), "Downloaded config version " + configuration.getVersion());


                initialize();
            } catch (Exception e) {
                Log.e(TAG, "FAILED TO DOWNLOAD CONFIG", e);
            }
        }).start();
    }

    private static BufferedInputStream getSheet(String sheet) throws IOException {
        return new BufferedInputStream(new URL(CONFIG_URL + sheet).openStream());
    }

    private static String binToString(BufferedInputStream in)  throws IOException {
        byte[] contents = new byte[1024];
        int bytesRead = 0;
        String strContents = "";
        while ((bytesRead = in.read(contents)) != -1) {
            strContents += new String(contents, 0, bytesRead);
        }
        return strContents;
    }

    public static Configuration getConfig() {
        return currentConfig;
    }

    public static String currentVersion() {
        return currentConfig.getVersion();
    }

}
