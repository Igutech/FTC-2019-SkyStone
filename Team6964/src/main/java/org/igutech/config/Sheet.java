package org.igutech.config;

import java.io.Serializable;
import java.util.HashMap;
import java.util.Map;

public class Sheet implements Serializable {

    private Map<String, Map<Integer, String>> sheet;

    /**
     * Parse a CSV to a sheet object
     * @param csvContent CSV content downloaded from google sheets
     */
    public Sheet(String csvContent) {
        sheet = new HashMap<>();


        String[] rows = csvContent.split("\n");
        int rowCount= 1;
        for (String row : rows) {
            String[] cols = row.split(",");
            char letter = 'A';
            for (String col : cols) {
                makeColIfNotExists(letter);
                if (col.length() >= 2) {
                    sheet.get(Character.toString(letter)).put(rowCount, col.substring(1, col.length()-1));
                } else {
                    sheet.get(Character.toString(letter)).put(rowCount, col);
                }
                if (letter == 'Z') break;
                letter++;
            }
            rowCount++;
        }
    }

    /**
     * Get cell content out of the sheet
     * @param col Column (letter)
     * @param row Row (number)
     * @return Cell content
     */
    public String get(String col, int row) {
        return sheet.get(col).get(row);
    }

    /**
     * Create a new column in the column map
     * @param letter
     */
    private void makeColIfNotExists(char letter) {
        if (!sheet.containsKey(Character.toString(letter))) {
            sheet.put(Character.toString(letter), new HashMap<Integer, String>());
        }
    }

    public Map<String, Map<Integer, String>> getContents() {
        return sheet;
    }

}
