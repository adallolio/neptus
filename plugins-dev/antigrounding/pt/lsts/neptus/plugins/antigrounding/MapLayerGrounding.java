/*
 * Copyright (c) 2004-2015 Norwegian University of Science and Technology (NTNU)
 * Centre for Autonomous Marine Operations and Systems (AMOS)
 * Department of Engineering Cybernetics (ITK)
 * All rights reserved.
 * O.S. Bragstads plass 2D, 7034 Trondheim, Norway
 *
 * This file is part of Neptus, Command and Control Framework.
 *
 * Commercial Licence Usage
 * Licencees holding valid commercial Neptus licences may use this file
 * in accordance with the commercial licence agreement provided with the
 * Software or, alternatively, in accordance with the terms contained in a
 * written agreement between you and Universidade do Porto. For licensing
 * terms, conditions, and further information contact lsts@fe.up.pt.
 *
 * Modified European Union Public Licence - EUPL v.1.1 Usage
 * Alternatively, this file may be used under the terms of the Modified EUPL,
 * Version 1.1 only (the "Licence"), appearing in the file LICENCE.md
 * included in the packaging of this file. You may not use this work
 * except in compliance with the Licence. Unless required by applicable
 * law or agreed to in writing, software distributed under the Licence is
 * distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF
 * ANY KIND, either express or implied. See the Licence for the specific
 * language governing permissions and limitations at
 * https://github.com/LSTS/neptus/blob/develop/LICENSE.md
 * and http://ec.europa.eu/idabc/eupl.html.
 *
 * For more information please see <http://lsts.fe.up.pt/neptus>.
 *
 * Author: Alberto Dallolio
 * Jan 28, 2020
 */
package pt.lsts.neptus.plugins.antigrounding;

import java.util.ArrayList;
import java.awt.Color;
import java.util.Collections;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.util.concurrent.TimeUnit;
import pt.lsts.neptus.util.AngleUtils;

import javax.swing.JMenuItem;
import javax.swing.JPopupMenu;


import com.google.common.eventbus.Subscribe;

import pt.lsts.imc.DeviceState;
import pt.lsts.imc.PlanSpecification;
import pt.lsts.imc.PlanGeneration;
import pt.lsts.imc.PlanGeneration.CMD;
import pt.lsts.imc.PlanGeneration.OP;
import pt.lsts.imc.PlanControl;
import pt.lsts.imc.PlanManeuver;
import pt.lsts.imc.Maneuver;
import pt.lsts.imc.PlanControlState;
import pt.lsts.imc.PlanControlState.STATE;
import pt.lsts.neptus.console.ConsoleLayout;
import pt.lsts.neptus.console.plugins.MainVehicleChangeListener;
import pt.lsts.neptus.console.ConsoleLayout;
import pt.lsts.neptus.console.ConsolePanel;
import pt.lsts.neptus.console.notifications.Notification;
import pt.lsts.neptus.gui.PropertiesEditor;
import pt.lsts.neptus.planeditor.IEditorMenuExtension;
import pt.lsts.neptus.planeditor.IMapPopup;
import pt.lsts.neptus.i18n.I18n;
import pt.lsts.neptus.plugins.NeptusProperty;
import pt.lsts.neptus.plugins.NeptusProperty.LEVEL;
import pt.lsts.neptus.plugins.PluginDescription;
import pt.lsts.neptus.plugins.SimpleRendererInteraction;
import pt.lsts.neptus.renderer2d.Renderer2DPainter;
import pt.lsts.neptus.renderer2d.StateRenderer2D;
import pt.lsts.neptus.types.coord.CoordinateUtil;
import pt.lsts.neptus.types.coord.LocationType;
import pt.lsts.neptus.util.logdb.SQLiteSerialization;
import java.util.LinkedHashMap;
import java.sql.Blob;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import pt.lsts.neptus.NeptusLog;
import pt.lsts.neptus.util.FileUtil;
import pt.lsts.neptus.util.conf.ConfigFetch;
import pt.lsts.neptus.types.mission.plan.PlanType;
import pt.lsts.neptus.comm.IMCUtils;

/**
 * @author alberto
 *
 */

@PluginDescription(name = "Antigrounding Layer", icon = "pt/lsts/neptus/plugins/antigrounding/icons/grounding.png", 
    author = "Alberto Dallolio", version = "0.1", description = "This plugin performs SQL database queries.")
public class MapLayerGrounding extends SimpleRendererInteraction implements Renderer2DPainter, MainVehicleChangeListener {

    // Simple settings

    @NeptusProperty(name = "Map grid size")
    public double grid_size = 50.0;

    @NeptusProperty(name = "Circle radius")
    public double circle_radius = 500.0;

    @NeptusProperty(name = "Square side")
    public double square_side = 200.0;
    
    @NeptusProperty(name = "Database path")
    String db_path = "ENCs/B1420_grid50_WGS84.db";

    private String currentPlan = null;
    private String currentManeuver = null;
    private PlanSpecification planSpec = null;


    // Advanced settings
    //@NeptusProperty(name = "Minimum Turn Radius", description = "Lateral turning radius of UAV (m).", 
    //        userLevel = LEVEL.ADVANCED, category = "Advanced")
    //public double minTurnRadius = 150;

    private LocationType landPos = null;
    

    private int[] arrX = { -8, -12, -12, 12, 12, 8, 0 };
    private int[] arrY = { 6, 6, 10, 10, 6, 6, -10 };
    private Polygon poly = new Polygon(arrX, arrY, 7);

    /**
     * @param console
     */
    public MapLayerGrounding(ConsoleLayout console) {
        super(console);
    }

    @Override
    public void initSubPanel() {
    }

    @Override
    public void cleanSubPanel() {
    }

    @Override
    public void setActive(boolean mode, StateRenderer2D source) {
        super.setActive(mode, source);
    }

    /**
     * On right-click, show popup menu on the map with plug-in options
     */
    @Override
    public void mouseClicked(MouseEvent event, StateRenderer2D source) {

        if (event.getButton() == MouseEvent.BUTTON3) {
            JPopupMenu popup = new JPopupMenu();
            final LocationType loc = source.getRealWorldLocation(event.getPoint());

            addDBConnect(popup);
            addGetSingleDepthMenu(popup, loc);
            addGetSquareMenu(popup, loc);
            addGetWithinRadiusMenu(popup, loc);
            addCurrentPlanMenu(popup);
            addSettingsMenu(popup);

            //addShowLocationsMenu(popup);
            //addShowPolygons(popup);

            popup.show(source, event.getPoint().x, event.getPoint().y);
        }
    }

    // I would like this to be inside a menu like "Connect to DB", but I cannot make it!
    public static final SQLiteSerialization ser = SQLiteSerialization.connect("ENCs/B1420_grid50_WGS84.db"); // use variable db_path
    // This function should be used.
    private void addDBConnect(JPopupMenu popup){
        
    }

    private void addGetSingleDepthMenu(JPopupMenu popup, final LocationType loc) {
        JMenuItem item = popup.add(I18n.text("Get single depth"));
        item.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                loc.convertToAbsoluteLatLonDepth();
                try {
                    double depth = getClosestDepth(loc.getLatitudeDegs(), loc.getLongitudeDegs(), grid_size); // exists: 63.322200, 10.169700
                } catch (Exception exc) {
                    // TODO: handle exception.
                }
            }
        });
    }

    private void addGetSquareMenu(JPopupMenu popup, final LocationType loc) {
        JMenuItem item = popup.add(I18n.text("Get square"));
        item.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                loc.convertToAbsoluteLatLonDepth();
                try {                    
                    ArrayList<ArrayList<Double>> cloud = getSquare(loc.getLatitudeDegs(), loc.getLongitudeDegs(), square_side/2);
                } catch (Exception exc) {
                    // TODO: handle exception.
                }
            }
        });
    }

    private void addGetWithinRadiusMenu(JPopupMenu popup, final LocationType loc) {
        JMenuItem item = popup.add(I18n.text("Get within radius"));
        item.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                loc.convertToAbsoluteLatLonDepth();
                try {                    
                    ArrayList<ArrayList<Double>> cloud = getWithinRadius(loc.getLatitudeDegs(), loc.getLongitudeDegs(), circle_radius);
                } catch (Exception exc) {
                    // TODO: handle exception.
                }
            }
        });
    }

    public double getClosestDepth(double lat, double lon, double grid_size) throws SQLException {
        ArrayList<Double> lats;
        ArrayList<Double> lons;
        ArrayList<Double> depths;
        double[] bear_range;
        double[] ranges = {0.0,0.0,0.0,0.0};
        
        LocationType queried = new LocationType(lat,lon);

        String statem = "select Depth from 'depthmap' where Lat = " + lat + " and Lon = " + lon + ";";
        double depth = 0.0;
        synchronized (ser.conn) {
            Statement st = ser.conn.createStatement();
            ResultSet rs = st.executeQuery(statem);
            if (!rs.isBeforeFirst() ) {
                System.out.println(" ----------- The query has produced no data! -------- ");
            }
            while(rs.next()) {
                depth = rs.getDouble(1);
            }
            rs.close();
        }

        // Meaning that the query has produced no data.
        if(depth==0.0)
        {
            ArrayList<ArrayList<Double>> four_points = getClosestDepths(lat, lon, grid_size);
            lats = four_points.get(0);
            lons = four_points.get(1);
            depths = four_points.get(2);

            for(int i=0; i<lats.size(); i++)
            {
                LocationType close = new LocationType(lats.get(i),lons.get(i));
                bear_range = CoordinateUtil.getNEBearingDegreesAndRange(queried,close);
                ranges[i] = bear_range[1];
                //NeptusLog.pub().info("BEARING " + bear_range[0] + " AND RANGE "  + bear_range[1]);
            }
            double[] min = getMin(ranges);
            //NeptusLog.pub().info("MIN RANGE " + min[0] + " ITS INDEX " + min[1]);
            depth = depths.get((int)min[1]);
            //NeptusLog.pub().info("CLOSEST DEPTH " + depth);
        }
        NeptusLog.pub().info("CLOSEST DEPTH " + depth);
        return depth;
    }

    public static double[] getMin(double[] inputArray){
        double minValue = inputArray[0];
        double[] ret = {0.0,0.0};
        double ind = 0.0;
        for(int i=1;i<inputArray.length;i++){ 
          if(inputArray[i] < minValue){ 
            minValue = inputArray[i];
            ind = i;
          }
        }
        ret[0] = minValue;
        ret[1] = ind;
        return ret;
    }

    public ArrayList<ArrayList<Double>> getClosestDepths(double Lat, double Lon, double grid_size) throws SQLException {
        double disp = 2*grid_size;

        ArrayList<ArrayList<Double>> all = new ArrayList<>();
        ArrayList<Double> lats = new ArrayList<>();
        ArrayList<Double> lons = new ArrayList<>();
        ArrayList<Double> depths = new ArrayList<>();
        
        //double lat_minus_displaced = Lat_rad;
        //WGS84::displace(-disp,0.0,&lat_minus_displaced, &Lon_rad);
        double [] lat_minus_disp = CoordinateUtil.WGS84displace(Lat, Lon, 0.0, -disp, 0.0, 0.0);
        double lat_minus_displaced = lat_minus_disp[0];
        //System.out.printf("LAT_D %f, LON_D %f, DEPTH_D %f\n", lat_minus_disp[0], lat_minus_disp[1], lat_minus_disp[2]);

        //double lat_plus_displaced = Lat_rad;
        //WGS84::displace(disp,0.0,&lat_plus_displaced, &Lon_rad);
        double [] lat_plus_disp = CoordinateUtil.WGS84displace(Lat, Lon, 0.0, disp, 0.0, 0.0);
        double lat_plus_displaced = lat_plus_disp[0];
        //System.out.printf("LAT_D %f, LON_D %f, DEPTH_D %f\n", lat_plus_disp[0], lat_plus_disp[1], lat_plus_disp[2]);

        //double lon_minus_displaced = Lon_rad;
        //WGS84::displace(0.0,-disp,&Lat_rad, &lon_minus_displaced);
        double [] lon_minus_disp = CoordinateUtil.WGS84displace(Lat, Lon, 0.0, 0.0, -disp, 0.0);
        double lon_minus_displaced = lon_minus_disp[1];
        //System.out.printf("LAT_D %f, LON_D %f, DEPTH_D %f\n", lon_minus_disp[0], lon_minus_disp[1], lon_minus_disp[2]);

        //double lon_plus_displaced = Lon_rad;
        //WGS84::displace(0.0,disp,&Lat_rad, &lon_plus_displaced);
        double [] lon_plus_disp = CoordinateUtil.WGS84displace(Lat, Lon, 0.0, 0.0, disp, 0.0);
        double lon_plus_displaced = lon_plus_disp[1];
        //System.out.printf("LAT_D %f, LON_D %f, DEPTH_D %f\n", lon_plus_disp[0], lon_plus_disp[1], lon_plus_disp[2]);

        String c_stmt = "select min(Lat+Lon), Lat, Lon, Depth from (select Lat, Lon, Depth from depthmap where Lat between " + lat_minus_displaced + " and " + lat_plus_displaced + " and Lon between " + lon_minus_displaced + " and " + lon_plus_displaced + ") where Lat >= " + Lat + " and Lon >= " + Lon +
        " union select max(Lat+Lon), Lat, Lon, Depth from (select Lat, Lon, Depth from depthmap where Lat between " + lat_minus_displaced + " and " + lat_plus_displaced + " and Lon between " + lon_minus_displaced + " and " + lon_plus_displaced + ") where Lat <= " + Lat + " and Lon <= " + Lon + 
        " union select min(Lat-Lon), Lat, Lon, Depth from (select Lat, Lon, Depth from depthmap where Lat between " + lat_minus_displaced + " and " + lat_plus_displaced + " and Lon between " + lon_minus_displaced + " and " + lon_plus_displaced + ") where Lat >= " + Lat + " and Lon <= " + Lon + 
        " union select max(Lat-Lon), Lat, Lon, Depth from (select Lat, Lon, Depth from depthmap where Lat between " + lat_minus_displaced + " and " + lat_plus_displaced + " and Lon between " + lon_minus_displaced + " and " + lon_plus_displaced + ") where Lat <= " + Lat + " and Lon >= " + Lon + ";";

        synchronized (ser.conn) {
            Statement st = ser.conn.createStatement();
            ResultSet rs = st.executeQuery(c_stmt);

            if (!rs.isBeforeFirst() ) {    
                System.out.println(" ----------- The query has produced no data! -------- ");
            }
            
            while(rs.next()) {
                //all.put(rs.getDouble(1), rs.getDouble(2));
                lats.add(rs.getDouble(2));
                lons.add(rs.getDouble(3));
                depths.add(rs.getDouble(4));
                //System.out.printf("%f, %f, %f\n", rs.getDouble(2), rs.getDouble(3), rs.getDouble(4));
            }
            rs.close();
        }
        all.add(lats);
        all.add(lons);
        all.add(depths);
        
        return all;
    }

    public ArrayList<ArrayList<Double>> getSquare(double Lat, double Lon, double half_size) throws SQLException {

        ArrayList<ArrayList<Double>> all = new ArrayList<>();
        ArrayList<Double> lats = new ArrayList<>();
        ArrayList<Double> lons = new ArrayList<>();
        ArrayList<Double> depths = new ArrayList<>();
        
        //double lat_minus_displaced = Lat_rad;
        //WGS84::displace(-half_size,0.0,&lat_minus_displaced, &Lon_rad);
        //lat_minus_displaced = Angles::degrees(lat_minus_displaced);
        double [] lat_minus_disp = CoordinateUtil.WGS84displace(Lat, Lon, 0.0, -half_size, 0.0, 0.0);
        double lat_minus_displaced = lat_minus_disp[0];
        System.out.printf("LAT_D_S %f, LON_D_S %f, DEPTH_D_S %f\n", lat_minus_disp[0], lat_minus_disp[1], lat_minus_disp[2]);

        //double lat_plus_displaced = Lat_rad;
        //WGS84::displace(half_size,0.0,&lat_plus_displaced, &Lon_rad);
        //lat_plus_displaced = Angles::degrees(lat_plus_displaced);
        double [] lat_plus_disp = CoordinateUtil.WGS84displace(Lat, Lon, 0.0, half_size, 0.0, 0.0);
        double lat_plus_displaced = lat_plus_disp[0];
        System.out.printf("LAT_D_S %f, LON_D_S %f, DEPTH_D_S %f\n", lat_plus_disp[0], lat_plus_disp[1], lat_plus_disp[2]);

        //double lon_minus_displaced = Lon_rad;
        //WGS84::displace(0.0,-half_size,&Lat_rad, &lon_minus_displaced);
        //lon_minus_displaced = Angles::degrees(lon_minus_displaced);
        double [] lon_minus_disp = CoordinateUtil.WGS84displace(Lat, Lon, 0.0, 0.0, -half_size, 0.0);
        double lon_minus_displaced = lon_minus_disp[1];
        System.out.printf("LAT_D_S %f, LON_D_S %f, DEPTH_D_S %f\n", lon_minus_disp[0], lon_minus_disp[1], lon_minus_disp[2]);

        //double lon_plus_displaced = Lon_rad;
        //WGS84::displace(0.0,half_size,&Lat_rad, &lon_plus_displaced);
        //lon_plus_displaced = Angles::degrees(lon_plus_displaced);
        double [] lon_plus_disp = CoordinateUtil.WGS84displace(Lat, Lon, 0.0, 0.0, half_size, 0.0);
        double lon_plus_displaced = lon_plus_disp[1];
        System.out.printf("LAT_D_S %f, LON_D_S %f, DEPTH_D_S %f\n", lon_plus_disp[0], lon_plus_disp[1], lon_plus_disp[2]);

        String c_stmt = "select Lat, Lon, Depth from depthmap where Lat between " + lat_minus_displaced + " and " + lat_plus_displaced + " and Lon between " + lon_minus_displaced + " and " + lon_plus_displaced + ";";
        System.out.printf("%s\n", c_stmt);

        synchronized (ser.conn) {
            Statement st = ser.conn.createStatement();
            ResultSet rs = st.executeQuery(c_stmt);

            if (!rs.isBeforeFirst()) {    
                System.out.println(" ----------- The query has produced no data! -------- ");
            }
            
            while(rs.next()) {
                //all.put(rs.getDouble(1), rs.getDouble(2));
                lats.add(rs.getDouble(2));
                lons.add(rs.getDouble(3));
                depths.add(rs.getDouble(4));
                System.out.printf("%f, %f, %f\n", rs.getDouble(2), rs.getDouble(3), rs.getDouble(4));
            }
            rs.close();
        }
        all.add(lats);
        all.add(lons);
        all.add(depths);

        return all;
    }

    public ArrayList<ArrayList<Double>> getWithinRadius(double Lat, double Lon, double radius) throws SQLException {
        ArrayList<Double> lats, lats_close = new ArrayList<>();
        ArrayList<Double> lons, lons_close = new ArrayList<>();
        ArrayList<Double> depths, depths_close = new ArrayList<>();
        ArrayList<ArrayList<Double>> ret = new ArrayList<ArrayList<Double>>(); // contains all the points (lat,lon,depth) within the radius.
        double[] bear_range;
        
        LocationType queried = new LocationType(Lat,Lon);

        ArrayList<ArrayList<Double>> square = getSquare(Lat,Lon,radius);
        lats = square.get(0);
        lons = square.get(1);
        depths = square.get(2);

        for(int i=0; i<lats.size(); i++)
        {
            LocationType close = new LocationType(lats.get(i),lons.get(i));
            bear_range = CoordinateUtil.getNEBearingDegreesAndRange(queried,close);
            if(bear_range[1] < radius)
            {
                lats_close.add(lats.get(i));
                lons_close.add(lons.get(i));
                depths_close.add(depths.get(i));
            }
        }
        ret.add(lats_close);
        ret.add(lons_close);
        ret.add(depths_close);

        return ret;
    }

    private void addSettingsMenu(JPopupMenu popup) {
        JMenuItem item = popup.add(I18n.text("Parameters"));
        item.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                PropertiesEditor.editProperties(MapLayerGrounding.this, getConsole(), true);
            }
        });
    }

    @Subscribe
    public void on(PlanSpecification msg) {
        NeptusLog.pub().info("PLAN SPECIFICATION REC");
        planSpec = msg;
    }

    /*@Subscribe
    public void on(PlanControl msg) {
        NeptusLog.pub().info("PATH CONTROL REC OUTER");
        if (msg.getSourceName().equals(getConsole().getMainSystem())) {
            NeptusLog.pub().info("PATH CONTROL REC INNERRRRRRRRRRRRRRRR");
            NeptusLog.pub().info("ARG " + msg.getMessage("arg").getAbbrev());
            if (msg.getMessage("arg").getAbbrev().equals("PlanSpecification")) {
                planSpec = (PlanSpecification) msg.getMessage("arg");
                NeptusLog.pub().info("PATH CONTROL REC INNER");
            }
        }
    }*/

    @Subscribe
    public void on(PlanControlState msg) {
        if (msg.getSourceName().equals(getConsole().getMainSystem())) {
            // if the vehicle is currently executing a plan we ask for that plan
            // and then identify what maneuver is being executed
            if (msg.getAsNumber("state").longValue() == STATE.EXECUTING.value()) {

                NeptusLog.pub().info("EXECUTING");

                //if (!msg.getAsString("plan_id").equals(currentPlan))
                //    samePlan = false;

                currentPlan = msg.getAsString("plan_id");
                currentManeuver = msg.getAsString("man_id");

                if (planSpec != null) {
                    for (PlanManeuver planMan : planSpec.getManeuvers()) {
                        Maneuver man = planMan.getData();
                        double lat = man.getAsNumber("lat").doubleValue();
                        double lon = man.getAsNumber("lon").doubleValue();
                        //if (planMan.getManeuverId().equals(currentManeuver)) {}

                        System.out.printf("LAAAAT %f, LOOOON %f\n", lat, lon);
                    }
                }
            }
        }   
    }

    private void addCurrentPlanMenu(JPopupMenu popup) {
        // First, check that connection to DB is on.

        //LinkedHashMap<Lat, Lon, Depth> all = new LinkedHashMap<Lat, Lon, Depth>();

    }


    /**
     * Always returns true
     */
    @Override
    public boolean isExclusive() {
        return true;
    }

    /**
     * Paints filled circles on the current target and drop positions.
     */
    /*@Override
    public void paint(Graphics2D g, StateRenderer2D renderer) {
        // If the land position has not been set, there is nothing to paint
        if (landPos == null)
            return;

        Point2D pt = renderer.getScreenPosition(landPos);
        g.translate(pt.getX(), pt.getY());

        // Draws the "arrow"
        g.setColor(Color.green);
        g.fillPolygon(poly);

        // Draws the "aiming point" in the middle
        g.setColor(Color.red);
        g.fill(new Ellipse2D.Double(-3, -3, 6, 6));
    }*/

    /*@Subscribe
    public void on(DeviceState state) {
        // Consumes changes to the net
        netHeading = Math.toDegrees(state.getPsi());
        double[] displaced = CoordinateUtil.WGS84displace(netLat, netLon, 0.0, state.getY(), state.getX(), 0.0);
        netLat = displaced[0];
        netLon = displaced[1];
        landPos.setLatitudeDegs(netLat);
        landPos.setLongitudeDegs(netLon);
        updateNetArrow();
    }*/


    /*private void updateNetArrow() {
        double angle = Math.toRadians(netHeading);
        for (int i = 0; i < poly.npoints; i++) {
            int x = arrX[i];
            int y = arrY[i];

            // Apply rotation
            double temp_x = x * Math.cos(angle) - y * Math.sin(angle);
            double temp_y = x * Math.sin(angle) + y * Math.cos(angle);

            poly.xpoints[i] = (int) Math.round(temp_x);
            poly.ypoints[i] = (int) Math.round(temp_y);
        }
    }*/
}