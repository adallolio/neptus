package pt.lsts.neptus.plugins.autonaut.l1.api;

public class RCC01 {
    public enum Field {
        RUDDER_ANGLE(0),
        THRUSTERS(1),
        R_KP(2),
        R_KI(3),
        POWER_SETTINGS(4),
        FALLBACK(5),
        MANUAL_CONTROL(6);

        public int v;

        Field(int i) {
            v = i;
        }
    }

    private String[] sentenceFields = new String[]{"0", "0", "0", "0", "000000", "0", "0"};

    public RCC01 set(RCC01.Field field, String value) {
        sentenceFields[field.v] = value;
        return this;
    }

        public String asNmeaSentence() {
        String sntc = "RCC01," + sentenceFields[Field.RUDDER_ANGLE.v] + "," +
                sentenceFields[Field.THRUSTERS.v] + "," +
                sentenceFields[Field.R_KP.v] + "," +
                sentenceFields[Field.R_KI.v] + "," +
                sentenceFields[Field.POWER_SETTINGS.v] + "," +
                sentenceFields[Field.FALLBACK.v] + "," +
                sentenceFields[Field.MANUAL_CONTROL.v];

        return "$" + sntc + "*"  + Integer.toHexString(checksum(sntc));
    }

    public String get(RCC01.Field fieldName) {
        return sentenceFields[fieldName.v];
    }

    private int checksum(String sentence) {
        int checksum = 0;
        for (char c : sentence.toCharArray()) {
            checksum ^= c;
        }

        return checksum;
    }
}
