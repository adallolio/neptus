package pt.lsts.neptus.plugins.autonaut.l1.api;


public class CR601 {
    public enum Field {
        LEAK_STATUS(0),
        POWER_SETTINGS(1),
        LOAD_POWER(2),
        PANEL_POWER(3),
        BATTERY(4),
        COG(5),
        SOG(6),
        N_SAT(7),
        LAT(8),
        LONG(9),
        KP(10),
        KI(11),
        E(12),
        INT_E(13),
        RUDDER(14),
        THRUST(15),
        L1_STATE(16),
        FALLBACK_MODE(17);

        public int v;

        Field(int i) {
            v = i;
        }
    }

    final String[] sentenceFields = new String[18];

    public CR601 set(CR601.Field field, String value) {
        sentenceFields[field.v] = value;
        return this;
    }

    public String get(CR601.Field field) {
        String v = sentenceFields[field.v];
        return (v == null) ? "" : v;
    }

    @Override
    public String toString() {
        int index = 0;
        return "leak: " + sentenceFields[index++] + " " +
                "power: " + sentenceFields[index++] + " " +
                "load: " + sentenceFields[index++] + " " +
                "panel: " + sentenceFields[index++] + " " +
                "battery: " + sentenceFields[index++] + " " +
                "cog: " + sentenceFields[index++] + " " +
                "sog: " + sentenceFields[index++] + " " +
                "num_sat: " + sentenceFields[index++] + " " +
                "latitude: " + sentenceFields[index++] + " " +
                "longitude: " + sentenceFields[index++] + " " +
                "Kp: " + sentenceFields[index++] + " " +
                "Ki: " + sentenceFields[index++] + " " +
                "error: " + sentenceFields[index++] + " " +
                "integrated_error: " + sentenceFields[index++] + " " +
                "applied_rudder: " + sentenceFields[index++] + " " +
                "applied_thrust: " + sentenceFields[index++] + " " +
                "l1 state: " + sentenceFields[index++] + " " +
                "fallback mode:  " + sentenceFields[index++];
    }
}