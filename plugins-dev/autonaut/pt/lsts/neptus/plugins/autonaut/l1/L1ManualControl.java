package pt.lsts.neptus.plugins.autonaut.l1;

import jssc.SerialPort;
import jssc.SerialPortEvent;
import jssc.SerialPortEventListener;
import jssc.SerialPortException;
import pt.lsts.neptus.NeptusLog;
import pt.lsts.neptus.console.ConsoleLayout;
import pt.lsts.neptus.console.ConsolePanel;
import pt.lsts.neptus.console.notifications.Notification;
import pt.lsts.neptus.plugins.NeptusProperty;
import pt.lsts.neptus.plugins.PluginDescription;
import pt.lsts.neptus.plugins.Popup;
import pt.lsts.neptus.plugins.autonaut.l1.api.CR601;
import pt.lsts.neptus.plugins.autonaut.l1.api.RCC01;
import pt.lsts.neptus.plugins.update.Periodic;
import pt.lsts.neptus.util.ImageUtils;

import javax.swing.*;
import java.awt.*;

@Popup(name = "L1 Manual Control", width = 550, height = 880)
@PluginDescription(name = "L1 Manual Control")
public class L1ManualControl extends ConsolePanel {


    @NeptusProperty(name = "Serial Port Device", category = "Serial Port")
    public String uartDevice = "/dev/ttyUSB4";

    @NeptusProperty(name = "Serial Port Baud Rate", category = "Serial Port")
    public int uartBaudRate = 38400;

    @NeptusProperty(name = "Serial Port Data Bits", category = "Serial Port")
    public int dataBits = 8;

    @NeptusProperty(name = "Serial Port Stop Bits", category = "Serial Port")
    public int stopBits = 1;

    @NeptusProperty(name = "Serial Port Parity Bits", category = "Serial Port")
    public int parity = 0;

    @NeptusProperty(name = "Rudder big increment", category = "Rudder")
    public int bigRudderIncrement = 15;

    @NeptusProperty(name = "Rudder small increment, in degrees", category = "Rudder")
    public int smallRudderIncrement = 9;

    @NeptusProperty(name = "Rudder actuation scaling factor", category = "Rudder")
    public int scalingFactor = 10;

    @NeptusProperty(name = "Maximum rudder angle", description = "Positive and negative. Without scaling factor", category = "Rudder")
    public int maxRudderAngle = 44;

    @NeptusProperty(name = "Thruster increment (in percentage)", category = "Thruster")
    public int thrusterIncrement = 20;

    @NeptusProperty(name = "Maximum thruster percentage", category = "Thruster")
    public int maxThrusterPercentage = 100;

    @NeptusProperty(name = "Kp increment", category = "Kp")
    public int kpIncrement = 1;

    @NeptusProperty(name = "Ki increment", category = "Ki")
    public double kiIncrement = 0.1;

    private final JPanel rudderStatePanel = new JPanel();
    private final JPanel ruddersButtonsPanel = new JPanel();
    private final JPanel thrustersButtonsPanel = new JPanel();
    private final JPanel fallbackButtonsPanel = new JPanel();
    private final JPanel msgPanel = new JPanel();
    private final JPanel monitorPanel = new JPanel();
    private final JPanel pwrSettingsPanel = new JPanel();
    private final JPanel gainsPanel = new JPanel();
    private final JLabel title = new JLabel("AutoNaut Control Interface");
    private final JLabel angleCommMsg = new JLabel("Commanded Rudder Angle: ");
    private final JLabel thrustCommMsg = new JLabel("Commanded Thrust: ");
    private final JLabel receivedMsg = new JLabel("Received: ");
    private final JLabel receivedCtndMsg = new JLabel();
    private final JLabel sentMsg = new JLabel("Sent: ");
    private final JLabel loadMsg = new JLabel("Load Power: ");
    private final JLabel panelsMsg = new JLabel("Panels Power: ");
    private final JLabel voltageMsg = new JLabel("Battery Voltage: ");
    private final JLabel cogMsg = new JLabel("COG: ");
    private final JLabel sogMsg = new JLabel("SOG: ");
    private final JLabel satMsg = new JLabel("Satellites: ");
    private final JLabel latMsg = new JLabel("LAT: ");
    private final JLabel longMsg = new JLabel("LONG: ");
    private final JLabel kpMsg = new JLabel("Kp: ");
    private final JLabel kiMsg = new JLabel("Ki: ");
    private final JLabel eMsg = new JLabel("Error: ");
    private final JLabel ieMsg = new JLabel("Integrated Error: ");
    private final JLabel angleAppMsg = new JLabel("Applied Rudded Angle: ");
    private final JLabel thrustAppMsg = new JLabel("Applied Thrust: ");

    private final JButton rudderBigLeftBtn = new JButton();
    private final JButton rudderBigRightBtn = new JButton();
    private final JButton rudderSmallLeftBtn = new JButton();
    private final JButton rudderSmallRightBtn = new JButton();
    private final JButton rudderCenterBtn = new JButton();
    private final JButton forwardBtn = new JButton();
    private final JButton stopBtn = new JButton();
    private final JButton revBtn = new JButton();
    private final JButton incKpBtn = new JButton("Kp+");
    private final JButton decKpBtn = new JButton("Kp-");
    private final JButton incKiBtn = new JButton("Ki+");
    private final JButton decKiBtn = new JButton("Ki-");
    private final JButton leakBtn = new JButton("LEAK");
    private final JToggleButton toggleFall1Btn = new JToggleButton("Rudder 0");
    private final JToggleButton toggleFall2Btn = new JToggleButton("Circle");
    private final JToggleButton toggleFall3Btn = new JToggleButton("Autopilot");

    private final JToggleButton toggleL2AisBtn = new JToggleButton("L2/AIS");
    private final JToggleButton toggleL3Btn = new JToggleButton("L3");
    //private final JToggleButton toggleGpsBtn = new JToggleButton("GPS L1");
    private final JToggleButton toggleIridiumBtn = new JToggleButton("IRIDIUM L1");
    private final JToggleButton toggleModSwiBtn = new JToggleButton("MOD/SWI");
    private final JToggleButton togglePumpsBtn = new JToggleButton("PUMPS");
    private final JToggleButton toggleOwlBtn = new JToggleButton("OWL VHF");

    private final JToggleButton toggleManualControlBtn = new JToggleButton();

    private SerialPort serialPort;

    private final CR601 currState = new CR601();
    private final RCC01 remoteCmd = new RCC01();
    private char[] looseBytes = new char[6];

    private boolean remoteCmdChanged = false;

    // Initial Autopilot gains
    //private double Kp = 0.1;
    //private double Ki = 1;

    private enum L1Messages
    {

        STATE("$CR601"),
        REMOTE_CTRL("$RCC01");

        private final String text;

        /**
         * @param text
         */
        L1Messages(final String text) {
            this.text = text;
        }

        /* (non-Javadoc)
         * @see java.lang.Enum#toString()
         */
        @Override
        public String toString() {
            return text;
        }
    }

    public L1ManualControl(ConsoleLayout console) {
        super(console);
        setupButtons();

        rudderStatePanel.setPreferredSize(new Dimension(550, 50));
        //rudderStatePanel.setBackground(Color.GRAY.darker());
        title.setFont(new Font("Arial", Font.PLAIN, 38));
        title.setForeground(Color.BLUE);
        rudderStatePanel.add(title);
        this.add(rudderStatePanel, BorderLayout.PAGE_START);

        thrustersButtonsPanel.setLayout(new BoxLayout(thrustersButtonsPanel, BoxLayout.PAGE_AXIS));
        thrustersButtonsPanel.setBorder(BorderFactory.createEmptyBorder(20, 0, 20, 0));
        thrustersButtonsPanel.add(forwardBtn);
        thrustersButtonsPanel.add(stopBtn);
        thrustersButtonsPanel.add(revBtn);
        this.add(thrustersButtonsPanel, BorderLayout.LINE_START);

        ruddersButtonsPanel.add(rudderBigLeftBtn);
        ruddersButtonsPanel.add(rudderSmallLeftBtn);
        ruddersButtonsPanel.add(rudderCenterBtn);
        ruddersButtonsPanel.add(rudderSmallRightBtn);
        ruddersButtonsPanel.add(rudderBigRightBtn);
        ruddersButtonsPanel.setBorder(BorderFactory.createEmptyBorder(20, 25, 20, 25));
        this.add(ruddersButtonsPanel, BorderLayout.CENTER);

        fallbackButtonsPanel.setLayout(new BoxLayout(fallbackButtonsPanel, BoxLayout.PAGE_AXIS));
        fallbackButtonsPanel.setBorder(BorderFactory.createEmptyBorder(20, 0, 20, 0));
        fallbackButtonsPanel.add(toggleFall1Btn);
        fallbackButtonsPanel.add(toggleFall2Btn);
        fallbackButtonsPanel.add(toggleFall3Btn);
        this.add(fallbackButtonsPanel, BorderLayout.LINE_END);

        pwrSettingsPanel.setLayout(new BoxLayout(pwrSettingsPanel, BoxLayout.PAGE_AXIS));
        pwrSettingsPanel.setBorder(BorderFactory.createEmptyBorder(20, 0, 20, 0));
        pwrSettingsPanel.add(toggleL2AisBtn);
        pwrSettingsPanel.add(toggleL3Btn);
        //pwrSettingsPanel.add(toggleGpsBtn);
        pwrSettingsPanel.add(toggleIridiumBtn);
        pwrSettingsPanel.add(toggleModSwiBtn);
        pwrSettingsPanel.add(togglePumpsBtn);
        pwrSettingsPanel.add(toggleOwlBtn);
        pwrSettingsPanel.add(leakBtn);
        this.add(pwrSettingsPanel, BorderLayout.LINE_START);

        monitorPanel.setLayout(new BoxLayout(monitorPanel, BoxLayout.PAGE_AXIS));
        monitorPanel.setBorder(BorderFactory.createEmptyBorder(0, 20, 0, 20));
        loadMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        loadMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        panelsMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        panelsMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        voltageMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        voltageMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        angleCommMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        angleCommMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        thrustCommMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        thrustCommMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        angleAppMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        angleAppMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        thrustAppMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        thrustAppMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        cogMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        cogMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        sogMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        sogMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        satMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        satMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        latMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        latMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        longMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        longMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        kpMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        kpMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        kiMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        kiMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        eMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        eMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        ieMsg.setFont(new Font("Arial", Font.PLAIN, 18));
        ieMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));

        monitorPanel.add(loadMsg);
        monitorPanel.add(panelsMsg);
        monitorPanel.add(voltageMsg);
        monitorPanel.add(angleCommMsg);
        monitorPanel.add(thrustCommMsg);
        monitorPanel.add(angleAppMsg);
        monitorPanel.add(thrustAppMsg);
        monitorPanel.add(cogMsg);
        monitorPanel.add(sogMsg);
        monitorPanel.add(satMsg);
        monitorPanel.add(latMsg);
        monitorPanel.add(longMsg);
        monitorPanel.add(kpMsg);
        monitorPanel.add(kiMsg);
        monitorPanel.add(eMsg);
        monitorPanel.add(ieMsg);
        this.add(monitorPanel, BorderLayout.CENTER);

        gainsPanel.setLayout(new BoxLayout(gainsPanel, BoxLayout.PAGE_AXIS));
        gainsPanel.setBorder(BorderFactory.createEmptyBorder(0, 5, 0, 5));
        gainsPanel.add(incKpBtn);
        gainsPanel.add(decKpBtn);
        gainsPanel.add(incKiBtn);
        gainsPanel.add(decKiBtn);
        this.add(gainsPanel, BorderLayout.LINE_END);
        
        msgPanel.setLayout(new BoxLayout(msgPanel, BoxLayout.PAGE_AXIS));
        msgPanel.setBorder(BorderFactory.createEmptyBorder(30, 20, 30, 20));
        receivedMsg.setFont(new Font("Arial", Font.PLAIN, 16));
        //receivedMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 5, 0));
        receivedCtndMsg.setFont(new Font("Arial", Font.PLAIN, 16));
        receivedCtndMsg.setBorder(BorderFactory.createEmptyBorder(0, 0, 10, 0));
        sentMsg.setFont(new Font("Arial", Font.PLAIN, 16));
        
        msgPanel.add(receivedMsg);
        msgPanel.add(receivedCtndMsg);
        msgPanel.add(sentMsg);
        this.add(msgPanel, BorderLayout.LINE_END);
        
        this.add(toggleManualControlBtn, BorderLayout.PAGE_START);
    }

    private void setupButtons() {
        final int imgSize = 40;
        final Image bigLeftImg = ImageUtils.getScaledImage(ImageUtils.getImage("pt/lsts/neptus/plugins/autonaut/l1/icons/left-h.png"),
                imgSize, imgSize, true);
        final Image leftImg = ImageUtils.getScaledImage(ImageUtils.getImage("pt/lsts/neptus/plugins/autonaut/l1/icons/left.png"),
                imgSize, imgSize, true);
        final Image bigRightImg = ImageUtils.getScaledImage(ImageUtils.getImage("pt/lsts/neptus/plugins/autonaut/l1/icons/right-h.png"),
                imgSize, imgSize, true);
        final Image rightImg = ImageUtils.getScaledImage(ImageUtils.getImage("pt/lsts/neptus/plugins/autonaut/l1/icons/right.png"),
                imgSize, imgSize, true);
        final Image toggleImg = ImageUtils.getScaledImage(ImageUtils.getImage("pt/lsts/neptus/plugins/autonaut/l1/icons/toggle.png"),
                imgSize, imgSize, true);
        final Image centerImg = ImageUtils.getScaledImage(ImageUtils.getImage("pt/lsts/neptus/plugins/autonaut/l1/icons/center.png"),
                imgSize, imgSize, true);
        final Image fwdImg = ImageUtils.getScaledImage(ImageUtils.getImage("pt/lsts/neptus/plugins/autonaut/l1/icons/fwd.png"),
                imgSize, imgSize, true);
        final Image revImg = ImageUtils.getScaledImage(ImageUtils.getImage("pt/lsts/neptus/plugins/autonaut/l1/icons/rev.png"),
                imgSize, imgSize, true);
        final Image stopImg = ImageUtils.getScaledImage(ImageUtils.getImage("pt/lsts/neptus/plugins/autonaut/l1/icons/stop.png"),
                imgSize, imgSize, true);

        rudderBigLeftBtn.setIcon(new ImageIcon(bigLeftImg));
        rudderSmallLeftBtn.setIcon(new ImageIcon(leftImg));
        rudderBigRightBtn.setIcon(new ImageIcon(bigRightImg));
        rudderSmallRightBtn.setIcon(new ImageIcon(rightImg));
        toggleManualControlBtn.setIcon(new ImageIcon(toggleImg));
        rudderCenterBtn.setIcon(new ImageIcon(centerImg));
        toggleFall1Btn.setFont(new Font("Arial", Font.PLAIN, 20));
        toggleFall2Btn.setFont(new Font("Arial", Font.PLAIN, 20));
        toggleFall3Btn.setFont(new Font("Arial", Font.PLAIN, 20));

        toggleL2AisBtn.setFont(new Font("Arial", Font.PLAIN, 20));
        toggleL3Btn.setFont(new Font("Arial", Font.PLAIN, 20));
        //toggleGpsBtn.setFont(new Font("Arial", Font.PLAIN, 20));
        toggleIridiumBtn.setFont(new Font("Arial", Font.PLAIN, 20));
        toggleModSwiBtn.setFont(new Font("Arial", Font.PLAIN, 20));
        togglePumpsBtn.setFont(new Font("Arial", Font.PLAIN, 20));
        toggleOwlBtn.setFont(new Font("Arial", Font.PLAIN, 20));
        leakBtn.setFont(new Font("Arial", Font.PLAIN, 20));

        incKpBtn.setFont(new Font("Arial", Font.PLAIN, 20));
        decKpBtn.setFont(new Font("Arial", Font.PLAIN, 20));
        incKiBtn.setFont(new Font("Arial", Font.PLAIN, 20));
        decKiBtn.setFont(new Font("Arial", Font.PLAIN, 20));

        forwardBtn.setIcon(new ImageIcon(fwdImg));
        revBtn.setIcon(new ImageIcon(revImg));
        stopBtn.setIcon(new ImageIcon(stopImg));

        toggleManualControlBtn.setBackground(Color.RED.darker());

        // Rudder button actions
        rudderCenterBtn.addActionListener(actionEvent -> setRudderValue(0));
        rudderSmallLeftBtn.addActionListener(actionEvent -> setRudderValue(-smallRudderIncrement * scalingFactor));
        rudderSmallRightBtn.addActionListener(actionEvent -> setRudderValue(smallRudderIncrement * scalingFactor));
        rudderBigLeftBtn.addActionListener(actionEvent -> setRudderValue(-bigRudderIncrement * scalingFactor));
        rudderBigRightBtn.addActionListener(actionEvent -> setRudderValue(bigRudderIncrement * scalingFactor));

        // Thruster actions
        forwardBtn.addActionListener(actionEvent -> setThrusterValue(thrusterIncrement));
        revBtn.addActionListener(actionEvent -> setThrusterValue(-thrusterIncrement));
        stopBtn.addActionListener(actionEvent -> setThrusterValue(0));

        // Fallback actions
        toggleFall1Btn.setEnabled(false);
        toggleFall1Btn.addActionListener(actionEvent -> toggleFall1());
        toggleFall2Btn.setEnabled(false);
        toggleFall2Btn.addActionListener(actionEvent -> toggleFall2());
        toggleFall3Btn.setEnabled(false);
        toggleFall3Btn.addActionListener(actionEvent -> toggleFall3());

        // Power Setting actions
        toggleL2AisBtn.setEnabled(false);
        toggleL2AisBtn.addActionListener(actionEvent -> toggleL2Ais());
        toggleL3Btn.setEnabled(false);
        toggleL3Btn.addActionListener(actionEvent -> toggleL3());
        //toggleGpsBtn.setEnabled(false);
        //toggleGpsBtn.addActionListener(actionEvent -> toggleGps());
        toggleIridiumBtn.setEnabled(false);
        toggleIridiumBtn.addActionListener(actionEvent -> toggleIridium());
        toggleModSwiBtn.setEnabled(false);
        toggleModSwiBtn.addActionListener(actionEvent -> toggleModSwi());
        togglePumpsBtn.setEnabled(false);
        togglePumpsBtn.addActionListener(actionEvent -> togglePumps());
        toggleOwlBtn.setEnabled(false);
        toggleOwlBtn.addActionListener(actionEvent -> toggleOwl());        

        // Gains Tuning Actions
        incKpBtn.addActionListener(actionEvent -> setKpValue(kpIncrement));
        decKpBtn.addActionListener(actionEvent -> setKpValue(-kpIncrement));
        incKiBtn.addActionListener(actionEvent -> setKiValue(kiIncrement));
        decKiBtn.addActionListener(actionEvent -> setKiValue(-kiIncrement));

        // Toggle actions
        toggleManualControlBtn.setEnabled(false);
        toggleManualControlBtn.addActionListener(actionEvent -> toggleManualControl());
    }

    private void setRudderValue(int value) {
        int newValue;
        if (value == 0)
            newValue = 0;
        else {
            try {
                newValue = Integer.valueOf(remoteCmd.get(RCC01.Field.RUDDER_ANGLE)) + value;
            } catch(NumberFormatException e) {
                newValue = value;
            }

            if (Math.abs(newValue) > Math.abs(maxRudderAngle * scalingFactor)) {
                getConsole().post(Notification.warning("L1 Manual Control", "Maximum rudder angle is " +
                        Math.abs(maxRudderAngle) +
                        " and trying to set " +
                        Math.abs(newValue / scalingFactor)));
                return;
            }
        }
        synchronized (remoteCmd) {
            String valueStr = String.valueOf(newValue);
            remoteCmd.set(RCC01.Field.RUDDER_ANGLE, valueStr);
            remoteCmdChanged = true;
            angleCommMsg.setText("Commanded Rudder Angle: " + Integer.valueOf(valueStr)/10 + " " + "°");
        }
    }

    private void setKpValue(int value) {
        int newValue;
        if (value == 0)
            newValue = 0;
        else {
            try {
                newValue = Integer.valueOf(remoteCmd.get(RCC01.Field.R_KP)) + value;
            } catch(NumberFormatException e) {
                newValue = value;
            }
            if (Math.abs(newValue) > 20 || newValue < 1) {
                getConsole().post(Notification.warning("L1 Manual Control", "Maximum Kp value is +-" +
                        20 +
                        " you're trying to set or going below zero " +
                        newValue));
                return;
            }
        }

        synchronized (remoteCmd) {
            String valueStr = String.valueOf(newValue);
            remoteCmd.set(RCC01.Field.R_KP, valueStr);
            remoteCmdChanged = true;
        }
    }

    private void setKiValue(double value) {
        double newValue;
        if (value == 0)
            newValue = 0;
        else {
            try {
                newValue = Double.valueOf(remoteCmd.get(RCC01.Field.R_KI)) + value;
            } catch(NumberFormatException e) {
                newValue = value;
            }

            if (Math.abs(newValue) > 20 || newValue < 0.1) {
                getConsole().post(Notification.warning("L1 Manual Control", "Maximum Ki value is +-" +
                        20 +
                        " you're trying to set or going below zero " +
                        newValue));
                return;
            }
        }
        synchronized (remoteCmd) {
            String valueStr = String.valueOf(newValue);
            remoteCmd.set(RCC01.Field.R_KI, valueStr);
            remoteCmdChanged = true;
        }
    }

    private void setThrusterValue(int value) {
        int newValue;
        if(value == 0)
            newValue = 0;
        else {
            try {
                newValue = Integer.valueOf(remoteCmd.get(RCC01.Field.THRUSTERS)) + value;
            } catch(NumberFormatException e) {
                newValue = value;
            }
            if (Math.abs(newValue) > Math.abs(maxThrusterPercentage)) {
                getConsole().post(Notification.warning("L1 Manual Control", "Maximum thruster value is " +
                        Math.abs(maxThrusterPercentage) +
                        " and trying to set " +
                        Math.abs(value / scalingFactor)));
                return;
            }
        }
        synchronized (remoteCmd) {
            String valueStr = String.valueOf(newValue);
            remoteCmd.set(RCC01.Field.THRUSTERS, valueStr);
            remoteCmdChanged = true;
            thrustCommMsg.setText("Commanded Thrust: " + String.valueOf(valueStr) + " " + "%");
        }
    }

    private void toggleManualControl() {
        String valueStr = toggleManualControlBtn.isSelected()  ? "1" : "0";
        remoteCmd.set(RCC01.Field.MANUAL_CONTROL, valueStr);
        // Set Initial PI Gains when take control.
        remoteCmd.set(RCC01.Field.R_KP, currState.get(CR601.Field.KP));
        remoteCmd.set(RCC01.Field.R_KI, currState.get(CR601.Field.KI));
        remoteCmdChanged = true;
    }

    private void toggleFall1() {
        getConsole().post(Notification.info("", "1"));
        toggleFall3Btn.setSelected(false);
        toggleFall2Btn.setSelected(false);

        synchronized (remoteCmd) {
            remoteCmd.set(RCC01.Field.FALLBACK, "0");
        }
    }

    private void toggleFall2() {
        getConsole().post(Notification.info("", "2"));
        toggleFall1Btn.setSelected(false);
        toggleFall3Btn.setSelected(false);

        String valueStr = toggleFall2Btn.isSelected()  ? "1" : "0";
        synchronized (remoteCmd) {
            remoteCmd.set(RCC01.Field.FALLBACK, valueStr);
        }
    }

    private void toggleFall3() {
        getConsole().post(Notification.info("", "3"));
        toggleFall1Btn.setSelected(false);
        toggleFall2Btn.setSelected(false);

        String valueStr = toggleFall3Btn.isSelected()  ? "2" : "0";
        synchronized (remoteCmd) {
            remoteCmd.set(RCC01.Field.FALLBACK, valueStr);
        }
    }


    private void toggleL2Ais() {
        getConsole().post(Notification.info("", "0"));
        toggleL3Btn.setSelected(false);
        //toggleGpsBtn.setSelected(false);
        toggleIridiumBtn.setSelected(false);
        toggleModSwiBtn.setSelected(false);
        togglePumpsBtn.setSelected(false);
        toggleOwlBtn.setSelected(false);

        char valueChar = toggleL2AisBtn.isSelected()  ? '1' : '0';
        synchronized (looseBytes) {
            looseBytes[0] = valueChar;
        }

        String str = new String(looseBytes);

        synchronized (remoteCmd) {
            remoteCmd.set(RCC01.Field.POWER_SETTINGS, str);
        }
    }

    private void toggleL3() {
        getConsole().post(Notification.info("", "0"));
        toggleL2AisBtn.setSelected(false);
        //toggleGpsBtn.setSelected(false);
        toggleIridiumBtn.setSelected(false);
        toggleModSwiBtn.setSelected(false);
        togglePumpsBtn.setSelected(false);
        toggleOwlBtn.setSelected(false);

        char valueChar = toggleL3Btn.isSelected()  ? '1' : '0';
        synchronized (looseBytes) {
            looseBytes[1] = valueChar;
        }
        String str = new String(looseBytes);
        synchronized (remoteCmd) {
            remoteCmd.set(RCC01.Field.POWER_SETTINGS, str);
        }
    }

/*
    private void toggleGps() {
        getConsole().post(Notification.info("", "0"));
        toggleL2AisBtn.setSelected(false);
        toggleL3Btn.setSelected(false);
        toggleIridiumBtn.setSelected(false);
        toggleModSwiBtn.setSelected(false);
        togglePumpsBtn.setSelected(false);
        toggleOwlBtn.setSelected(false);

        char valueChar = toggleGpsBtn.isSelected()  ? '1' : '0';
        synchronized (looseBytes) {
            looseBytes[2] = valueChar;
        }
        String str = new String(looseBytes);
        synchronized (remoteCmd) {
            remoteCmd.set(RCC01.Field.POWER_SETTINGS, str);
        }
    }
*/

    private void toggleIridium() {
        getConsole().post(Notification.info("", "0"));
        toggleL2AisBtn.setSelected(false);
        toggleL3Btn.setSelected(false);
        //toggleGpsBtn.setSelected(false);
        toggleModSwiBtn.setSelected(false);
        togglePumpsBtn.setSelected(false);
        toggleOwlBtn.setSelected(false);

        char valueChar = toggleIridiumBtn.isSelected()  ? '1' : '0';
        synchronized (looseBytes) {
            looseBytes[2] = valueChar;
        }
        String str = new String(looseBytes);
        synchronized (remoteCmd) {
            remoteCmd.set(RCC01.Field.POWER_SETTINGS, str);
        }
    }

    private void toggleModSwi() {
        getConsole().post(Notification.info("", "0"));
        toggleL2AisBtn.setSelected(false);
        toggleL3Btn.setSelected(false);
        //toggleGpsBtn.setSelected(false);
        toggleIridiumBtn.setSelected(false);
        togglePumpsBtn.setSelected(false);

        char valueChar = toggleModSwiBtn.isSelected()  ? '1' : '0';
        synchronized (looseBytes) {
            looseBytes[3] = valueChar;
        }
        String str = new String(looseBytes);
        synchronized (remoteCmd) {
            remoteCmd.set(RCC01.Field.POWER_SETTINGS, str);
        }
    }

    private void togglePumps() {
        getConsole().post(Notification.info("", "0"));
        toggleL2AisBtn.setSelected(false);
        toggleL3Btn.setSelected(false);
        //toggleGpsBtn.setSelected(false);
        toggleModSwiBtn.setSelected(false);
        toggleIridiumBtn.setSelected(false);
        toggleOwlBtn.setSelected(false);

        char valueChar = togglePumpsBtn.isSelected()  ? '1' : '0';
        synchronized (looseBytes) {
            looseBytes[4] = valueChar;
        }
        String str = new String(looseBytes);
        synchronized (remoteCmd) {
            remoteCmd.set(RCC01.Field.POWER_SETTINGS, str);
        }
    }

    private void toggleOwl() {
        getConsole().post(Notification.info("", "0"));
        toggleL2AisBtn.setSelected(false);
        toggleL3Btn.setSelected(false);
        //toggleGpsBtn.setSelected(false);
        toggleModSwiBtn.setSelected(false);
        toggleIridiumBtn.setSelected(false);
        togglePumpsBtn.setSelected(false);

        char valueChar = toggleOwlBtn.isSelected()  ? '1' : '0';
        synchronized (looseBytes) {
            looseBytes[5] = valueChar;
        }
        String str = new String(looseBytes);
        synchronized (remoteCmd) {
            remoteCmd.set(RCC01.Field.POWER_SETTINGS, str);
        }
    }


    private void connectToSerial() throws Exception {
        serialPort = new SerialPort(uartDevice);

        boolean open = serialPort.openPort();
        if (!open) {
            serialPort = null;
            Exception e = new Exception("Unable to open port " + uartDevice);
            throw e;
        }

        serialPort.setParams(uartBaudRate, dataBits, stopBits, parity);
        serialPort.addEventListener(new SerialPortEventListener() {
            String currString;
            @Override
            public void serialEvent(SerialPortEvent serEvt) {
                String bfr;
                try {
                    bfr = serialPort.readString();
                    // all cases append string
                    currString += bfr;

                    // we're finished
                    if (bfr.contains("*")) {
                        parseNmea(currString);
                        currString = "";

                        boolean manualControl = false;
                        synchronized (remoteCmd) {
                            if (remoteCmd.get(RCC01.Field.MANUAL_CONTROL).equals("1")) {
                                dispatchRemoteControlCommand(remoteCmd);
                                manualControl = true;
                            }
                        }

                        if(!manualControl)
                            updateRemoteCommand();
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });
    }

    private void updateRemoteCommand() {
        remoteCmd.set(RCC01.Field.POWER_SETTINGS, currState.get(CR601.Field.POWER_SETTINGS))
                .set(RCC01.Field.FALLBACK, currState.get(CR601.Field.FALLBACK_MODE));
    }


    private void parseNmea(String sentence) {
        NeptusLog.pub().info(sentence);
        //TODO validate checksum
        String[] parts = sentence.substring(0, sentence.lastIndexOf("*")).split(",");
        //NeptusLog.pub().info(parts[0]);
        
        // Convert GPS raw lat/long into human-readable
        double lat_coords;
        double long_coords;
        try {
            lat_coords = Double.parseDouble(parts[9]); //"100.1"
            long_coords = Double.parseDouble(parts[10]); //"100.1"
        }
        catch(Exception e) {
            lat_coords = Double.parseDouble("100.1");
            long_coords = Double.parseDouble("100.1");
        }
        
        double degValueLat = lat_coords / 100;
        double degValueLong = long_coords / 100;
        int degreesLat = (int) degValueLat;
        int degreesLong = (int) degValueLong;
        double decMinutesSecondsLat = ((degValueLat - degreesLat)) / .60;
        double decMinutesSecondsLong = ((degValueLong - degreesLong)) / .60;
        double minuteValueLat = decMinutesSecondsLat * 60;
        double minuteValueLong = decMinutesSecondsLong * 60;
        int minutesLat = (int) minuteValueLat;
        int minutesLong = (int) minuteValueLong;
        double secsValueLat = (minuteValueLat - minutesLat) * 60;
        double secsValueLong = (minuteValueLong - minutesLong) * 60;


        synchronized(looseBytes){
            looseBytes = parts[2].toCharArray();

            if(looseBytes[0]=='1'){
                    toggleL2AisBtn.setForeground(Color.RED.darker());
            } else {
                toggleL2AisBtn.setForeground(new JButton().getForeground());
            }
            if(looseBytes[1]=='1'){
                    toggleL3Btn.setForeground(Color.RED.darker());
            } else {
                toggleL3Btn.setForeground(new JButton().getForeground());
            }
/*            if(looseBytes[2]=='1'){
                    toggleGpsBtn.setForeground(Color.RED.darker());
            } else {
                toggleGpsBtn.setForeground(new JButton().getForeground());
            }*/
            if(looseBytes[2]=='1'){
                    toggleIridiumBtn.setForeground(Color.RED.darker());
            } else {
                toggleIridiumBtn.setForeground(new JButton().getForeground());
            }
            if(looseBytes[3]=='1'){
                    toggleModSwiBtn.setForeground(Color.RED.darker());
            } else {
                toggleModSwiBtn.setForeground(new JButton().getForeground());
            }
            if(looseBytes[4]=='1'){
                    togglePumpsBtn.setForeground(Color.RED.darker());
            } else {
                togglePumpsBtn.setForeground(new JButton().getForeground());
            }
            if(looseBytes[5]=='1'){
                    toggleOwlBtn.setForeground(Color.RED.darker());
            } else {
                toggleOwlBtn.setForeground(new JButton().getForeground());
            }
        }

        char[] leak = parts[1].toCharArray();
        if(leak[0]=='1'){
            leakBtn.setForeground(Color.RED.darker());
        } else {
            leakBtn.setForeground(new JButton().getForeground());
        }

        char[] fallback = parts[18].toCharArray();
        if(fallback[0]=='0'){
                    toggleFall1Btn.setForeground(Color.RED.darker());
                    toggleFall2Btn.setForeground(new JButton().getForeground());
                    toggleFall3Btn.setForeground(new JButton().getForeground());
            } else if(fallback[0]=='1') {
                    toggleFall2Btn.setForeground(Color.RED.darker());
                    toggleFall1Btn.setForeground(new JButton().getForeground());
                    toggleFall3Btn.setForeground(new JButton().getForeground());
            } else if(fallback[0]=='2') {
                    toggleFall3Btn.setForeground(Color.RED.darker());
                    toggleFall1Btn.setForeground(new JButton().getForeground());
                    toggleFall2Btn.setForeground(new JButton().getForeground());
            }

        if (parts[0].equals(L1Messages.STATE.text)) {
            synchronized (currState) {
                currState.set(CR601.Field.LEAK_STATUS, parts[1])
                .set(CR601.Field.POWER_SETTINGS, parts[2])
                .set(CR601.Field.LOAD_POWER, parts[3])
                .set(CR601.Field.PANEL_POWER, parts[4])
                .set(CR601.Field.BATTERY, parts[5])
                .set(CR601.Field.COG, parts[6])
                .set(CR601.Field.SOG, parts[7])
                .set(CR601.Field.N_SAT, parts[8])
                .set(CR601.Field.LAT, parts[9])
                .set(CR601.Field.LONG, parts[10])
                .set(CR601.Field.KP, parts[11])
                .set(CR601.Field.KI, parts[12])
                .set(CR601.Field.E, parts[13])
                .set(CR601.Field.INT_E, parts[14])
                .set(CR601.Field.RUDDER, parts[15])
                .set(CR601.Field.THRUST, parts[16])
                .set(CR601.Field.L1_STATE, parts[17])
                .set(CR601.Field.FALLBACK_MODE, parts[18]);

                //NeptusLog.pub().info(sentence);
                //receivedMsg.setText("R: " + String.valueOf(sentence));
                receivedMsg.setText("Received: " + String.valueOf(parts[0]) + "," + String.valueOf(parts[1]) + "," + String.valueOf(parts[2]) + "," + String.valueOf(parts[3]) + "," + String.valueOf(parts[4]) + "," + String.valueOf(parts[5]) + "," + String.valueOf(parts[6]) + "," + String.valueOf(parts[7]) + "," + String.valueOf(parts[8]));
                receivedCtndMsg.setText("                   " + String.valueOf(parts[9]) + "," + String.valueOf(parts[10]) + "," + String.valueOf(parts[11]) + "," + String.valueOf(parts[12]) + "," + String.valueOf(parts[13]) + "," + String.valueOf(parts[14]) + "," + String.valueOf(parts[15]) + "," + String.valueOf(parts[16]) + "," + String.valueOf(parts[17]) + "," + String.valueOf(parts[18]));
                loadMsg.setText("Load Power: " + String.valueOf(parts[3]) + " " + "W");
                panelsMsg.setText("Panels Power: " + String.valueOf(parts[4]) + " " + "W");
                voltageMsg.setText("Battery Voltage: " + String.valueOf(parts[5]) + " " + "V");
                satMsg.setText("Satellites: " + String.valueOf(parts[8]));
                cogMsg.setText("COG: " + String.valueOf(parts[6]) + " " + "°");
                sogMsg.setText("SOG: " + String.valueOf(parts[7]) + " " + "kn");
                latMsg.setText("LAT: " + degreesLat + "\u00B0" + " " + minutesLat + "' " + String.format("%.1f", secsValueLat) + "\" ");
                longMsg.setText("LONG: " + degreesLong + "\u00B0" + " " + minutesLong + "' " + String.format("%.1f", secsValueLong) + "\" ");
                kpMsg.setText("Kp: " + String.valueOf(parts[11]));
                kiMsg.setText("Ki: " + String.valueOf(parts[12]));
                eMsg.setText("Error: " + String.valueOf(parts[13]));
                ieMsg.setText("Integrated Error: " + String.valueOf(parts[14]));
                angleAppMsg.setText("Applied Rudder Angle: " + Integer.valueOf(parts[15])/10 + " " + "°");
                thrustAppMsg.setText("Applied Thrust: " + String.valueOf(parts[16]) + " " + "%");

                //angleMsg.setText("Applied Rudder Angle: " + String.valueOf(parts[BO]) + " " + "°")
            }
        }
        //NeptusLog.pub().info(sentence);
    }

    private void dispatchRemoteControlCommand(RCC01 cmd) {
        String nmeaCmd = cmd.asNmeaSentence();
        NeptusLog.pub().info(nmeaCmd);
        sentMsg.setText("Sent: " + String.valueOf(nmeaCmd));

        try {
            serialPort.writeString(nmeaCmd + "\r\n");
        } catch (SerialPortException e) {
            NeptusLog.pub().error(e.getMessage());
            getConsole().post(Notification.error("L1 Manual Control", "Failed to send " + nmeaCmd));
        }

        remoteCmdChanged = false;
    }

    private void toggleControlPanel(boolean value) {
        rudderSmallLeftBtn.setEnabled(value);
        rudderBigLeftBtn.setEnabled(value);
        rudderBigRightBtn.setEnabled(value);
        rudderSmallRightBtn.setEnabled(value);
        rudderCenterBtn.setEnabled(value);
        forwardBtn.setEnabled(value);
        revBtn.setEnabled(value);
        stopBtn.setEnabled(value);
        toggleManualControlBtn.setEnabled(value);
        toggleFall1Btn.setEnabled(value);
        toggleFall2Btn.setEnabled(value);
        toggleFall3Btn.setEnabled(value);
        toggleL2AisBtn.setEnabled(value);
        toggleL3Btn.setEnabled(value);
        //toggleGpsBtn.setEnabled(value);
        toggleIridiumBtn.setEnabled(value);
        toggleModSwiBtn.setEnabled(value);
        togglePumpsBtn.setEnabled(value);
        toggleOwlBtn.setEnabled(value);
    }

    @Override
    public void cleanSubPanel() {
        try {
            serialPort.closePort();
        } catch (SerialPortException e) {
            NeptusLog.pub().debug(e.getMessage());
            NeptusLog.pub().warn("Couldn't close " + uartDevice + " port.");
        }
    }

    @Periodic(millisBetweenUpdates = 3000)
    public void onPeriodic() {
        try {
            if (serialPort == null || !serialPort.isOpened()) {
                connectToSerial();
                toggleControlPanel(true);
            }
        } catch (Exception e) {
            NeptusLog.pub().error(e);
            getConsole().post(Notification.error("L1 Manual Control",
                    "Error connecting via serial to  \"" + serialPort + "\".").requireHumanAction(false));
            toggleControlPanel(false);
        }
    }

    @Override
    public void initSubPanel() {
    }
}
