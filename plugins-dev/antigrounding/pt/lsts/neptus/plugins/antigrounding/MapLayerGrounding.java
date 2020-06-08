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

import java.util.*;
import java.util.ArrayList;
import java.awt.Color;
import java.awt.Font;
import java.util.Collections;
import java.awt.Graphics2D;
import java.awt.Dimension;
import java.awt.Polygon;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import pt.lsts.neptus.colormap.ColorBar;
import pt.lsts.neptus.colormap.ColorBarPainterUtil;
import pt.lsts.neptus.colormap.ColorMap;
import pt.lsts.neptus.colormap.ColorMapFactory;
import java.awt.event.MouseEvent;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.awt.geom.Point2D;
import java.util.concurrent.TimeUnit;
import pt.lsts.neptus.util.AngleUtils;
import pt.lsts.neptus.util.ImageUtils;
import pt.lsts.neptus.plugins.PluginUtils;

import javax.swing.JMenu;
import javax.swing.JMenuItem;
import javax.swing.JPopupMenu;
import javax.swing.JMenuBar;
import pt.lsts.neptus.mystate.MyState;


import com.google.common.eventbus.Subscribe;

import pt.lsts.imc.DeviceState;
import pt.lsts.imc.PlanSpecification;
import pt.lsts.imc.PlanGeneration;
import pt.lsts.imc.PlanDB;
import pt.lsts.imc.GpsFix;
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
import javax.swing.JOptionPane;
import javax.swing.JDialog;
import javax.swing.JComboBox;
import javax.swing.JTextField;
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
import java.awt.Image;
import javax.swing.ImageIcon;
import pt.lsts.neptus.util.FileUtil;
import pt.lsts.neptus.util.conf.ConfigFetch;
import pt.lsts.neptus.types.mission.plan.PlanType;
import pt.lsts.neptus.comm.IMCUtils;
import java.io.File;
import javax.imageio.ImageIO;

/**
 * @author alberto
 *
 */

@PluginDescription(name = "Antigrounding Layer", icon = "pt/lsts/neptus/plugins/antigrounding/icons/antigrounding.png", 
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
    //public String db_path = "ENCs/B1420_grid50_WGS84.db";
    //public String db_path = "ENCs/poi_depthmap_depare-rad-indexed.db";
    static public String db_path = "ENCs/Nordfjord.db";

    @NeptusProperty(name = "Buoys layer name", description = "Buoys layer name", 
            userLevel = LEVEL.ADVANCED, category = "Advanced")
    public String buoys_layer = "BOYCAR";

    @NeptusProperty(name = "Beacons layer name", description = "Beacons layer name", 
            userLevel = LEVEL.ADVANCED, category = "Advanced")
    public String beacons_layer = "BCN";

    @NeptusProperty(name = "Dangers layer name", description = "Generally, an area where the mariner has to be made aware of circumstances influencing the safety of navigation.", 
            userLevel = LEVEL.ADVANCED, category = "Advanced")
    public String dangers_layer = "CTNARE";

    @NeptusProperty(name = "Regime layer name", description = "Area subject to a specific legal regime established in the United Nations Convention on the Law of the Sea under which the coastal state has certain rights and jurisdiction.", 
            userLevel = LEVEL.ADVANCED, category = "Advanced")
    public String regimes_layer = "EXEZNE";

    @NeptusProperty(name = "Lights layer name", description = "A luminous or lighted aid to navigation.", 
            userLevel = LEVEL.ADVANCED, category = "Advanced")
    public String lights_layer = "LIGHTS";

    @NeptusProperty(name = "Navigation Line layer name", description = "A navigation line is a straight line extending towards an area of navigational interest and generally generated by two navigational aids.", 
            userLevel = LEVEL.ADVANCED, category = "Advanced")
    public String navlines_layer = "NAVLNE";

    @NeptusProperty(name = "Obstruction layer name", description = "Anything that hinders or prevents movement, particularly anything that endangers or prevents passage of a vessel.", 
            userLevel = LEVEL.ADVANCED, category = "Advanced")
    public String obstructions_layer = "OBSTRN";

    @NeptusProperty(name = "Piers layer name", description = "A long heavy timber or section of steel, wood, concrete, etc.. forced into the earth which may serve as a support, as for a pier, or a free standing pole within a marine environment.", 
            userLevel = LEVEL.ADVANCED, category = "Advanced")
    public String piers_layer = "PILPNT";

    @NeptusProperty(name = "Depth Contour layer name", description = "Depth contours.", 
            userLevel = LEVEL.ADVANCED, category = "Advanced")
    public String depth_cont_layer = "DEPARE";

    @NeptusProperty(name = "Rocks layer name", description = "A concreted mass of stony material or coral which dries, is awash or is below the water surface.", 
            userLevel = LEVEL.ADVANCED, category = "Advanced")
    public String stones_layer = "UWTROC";

    @NeptusProperty(name = "Wrecks layer name", description = "The ruined remains of a stranded or sunken vessel which has been rendered useless.", 
            userLevel = LEVEL.ADVANCED, category = "Advanced")
    public String wrecks_layer = "WRECKS";

    //public static final Color VERY_DARK_RED = new Color(153,0,0);

    private boolean samePlan = false;
    private String newPlan = null;
    private String currentPlan = null;
    private String currentManeuver = null;
    private PlanSpecification planSpec = null;
    public ArrayList<Double> current_targets_lat = new ArrayList<Double>();
    public ArrayList<Double> current_targets_lon = new ArrayList<Double>();
    public boolean show_legend = false;
    public boolean show_grounding = false;
    public boolean show_soundings = false;
    public boolean show_waypoints = false;
    public boolean show_single = false;
    public boolean request_single = false;
    public boolean show_square = false;
    public boolean show_circle = false;
    public boolean show_buoys = false;
    public boolean show_depth_cont = false;
    public boolean show_beacons = false;
    public boolean show_dangers = false;
    public boolean show_lights = false;
    public boolean show_navlines = false;
    public boolean show_obstructions = false;
    public boolean show_piers = false;
    public boolean show_regimes = false;
    public boolean show_wrecks = false;
    public boolean show_stones = false;
    public boolean second_wp = false;
    public boolean check_transect = false;
    public boolean show_single_transect = false;
    public boolean show_transects = false;
    public ArrayList<Double> plan_waypoint = new ArrayList<Double>();
    public ArrayList<ArrayList<Double>> plan_waypoints = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> transects = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> single_transect = new ArrayList<ArrayList<Double>>();
    LocationType autonaut = new LocationType();
    LocationType transect_start = new LocationType();
    LocationType transect_end = new LocationType();
    float radius_depare = 0.0f;
    public LocationType mouse_click = new LocationType();
    private ColorMap colorMap = ColorMapFactory.createBlueToRedColorMap();
    ArrayList<ArrayList<Double>> singles = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> square = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> circle = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> ground = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> buoys = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> beacons = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> depth_conts = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> dangers = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> lights = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> navlines = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> obstructions = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> piers = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> regimes = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> wrecks = new ArrayList<ArrayList<Double>>();
    ArrayList<ArrayList<Double>> stones = new ArrayList<ArrayList<Double>>();
    public ArrayList<LocationType> regimes_vertices = new ArrayList<LocationType>();
    public ArrayList<Double> depth_contour = new ArrayList<Double>();

    public static final Color DARK_BLUE = new Color(0,0,204);

    private List<Image> images = new ArrayList<Image>();

    public String current_dir = System.getProperties().getProperty("user.dir") + "/plugins-dev/antigrounding/pt/lsts/neptus/plugins/antigrounding/";

    //public String[] contour_km = {"1","2","5","10","20"};
    //public String[] drval2 = {"5","10","20"};
    //public JComboBox jc = new JComboBox(contour_km);
    //public JComboBox jd = new JComboBox(drval2);


    // Advanced settings
    //@NeptusProperty(name = "Minimum Turn Radius", description = "Lateral turning radius of UAV (m).", 
    //        userLevel = LEVEL.ADVANCED, category = "Advanced")
    //public double minTurnRadius = 150;

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

        images.add(new ImageIcon(current_dir+"icons/buoy.png").getImage().getScaledInstance(30, 30, Image.SCALE_SMOOTH));
        images.add(new ImageIcon(current_dir+"icons/beacon.png").getImage().getScaledInstance(30, 30, Image.SCALE_SMOOTH));
        images.add(new ImageIcon(current_dir+"icons/light.png").getImage().getScaledInstance(30, 30, Image.SCALE_SMOOTH));
        images.add(new ImageIcon(current_dir+"icons/obstruction.png").getImage().getScaledInstance(30, 30, Image.SCALE_SMOOTH));
        images.add(new ImageIcon(current_dir+"icons/pier.png").getImage().getScaledInstance(30, 30, Image.SCALE_SMOOTH));
        images.add(new ImageIcon(current_dir+"icons/rock.png").getImage().getScaledInstance(30, 30, Image.SCALE_SMOOTH));
        images.add(new ImageIcon(current_dir+"icons/wreck.png").getImage().getScaledInstance(30, 30, Image.SCALE_SMOOTH));
        images.add(new ImageIcon(current_dir+"icons/grounding.png").getImage().getScaledInstance(30, 30, Image.SCALE_SMOOTH));
        images.add(new ImageIcon(current_dir+"icons/danger.png").getImage().getScaledInstance(30, 30, Image.SCALE_SMOOTH));

        if (event.getButton() == MouseEvent.BUTTON3) {
            JPopupMenu popup = new JPopupMenu();
            final LocationType loc = source.getRealWorldLocation(event.getPoint());
            mouse_click.setLocation(loc);

            //addDBConnect(popup);
            addGetSingleDepthMenu(popup, loc);
            addGetTransectMenu(popup);
            addGetSquareMenu(popup, loc);
            addGetWithinRadiusMenu(popup, loc);
            popup.addSeparator();
            addCurrentPlanMenu(popup);
            addShowWaypointsMenu(popup);
            addShowTransectMenu(popup);
            popup.addSeparator();
            addShowFeaturesMenu(popup);
            popup.addSeparator();
            addShowSoundingsMenu(popup);
            addShowLegendMenu(popup);
            addSettingsMenu(popup);

            //addShowLocationsMenu(popup);
            //addShowPolygons(popup);

            popup.show(source, event.getPoint().x, event.getPoint().y);
        }

        if(show_single && event.getButton() == MouseEvent.BUTTON1)
        {
            LocationType loc = source.getRealWorldLocation(event.getPoint());
            loc.convertToAbsoluteLatLonDepth();
            try {
                ArrayList<Double> single = getClosestDepth(loc.getLatitudeRads(), loc.getLongitudeRads(), grid_size);
                if(single.isEmpty())
                    NeptusLog.pub().info("grounding");
                else
                    singles.add(single);
            } catch (Exception exc) {
                // TODO: handle exception.
            }
        }

        if (event.getButton() == MouseEvent.BUTTON1 && check_transect) {
            LocationType loc = source.getRealWorldLocation(event.getPoint());
            loc.convertToAbsoluteLatLonDepth();
            transect_start.setLocation(loc);
            second_wp = true;
            check_transect = false;
        }else if (event.getButton() == MouseEvent.BUTTON1 && second_wp) {
            LocationType loc = source.getRealWorldLocation(event.getPoint());
            loc.convertToAbsoluteLatLonDepth();
            transect_end.setLocation(loc);
            second_wp = false;

            // Check this transect.
            try {
                single_transect.clear();
                single_transect = checkTransect(transect_start,transect_end);
                show_single_transect = true;
            } catch (Exception exc) {
                // TODO: handle exception.
            }
        }
    }

    // I would like this to be inside a menu like "Connect to DB", but I cannot make it!
    public static final SQLiteSerialization ser = SQLiteSerialization.connect(db_path); // use variable db_path "ENCs/poi_depthmap_depare-rad-indexed.db"
    // This function should be used.
    //private void addDBConnect(JPopupMenu popup){
        
    //}

    private void addGetSingleDepthMenu(JPopupMenu popup, final LocationType loc) {
        if(!show_single && !show_grounding)
        {
            popup.add(I18n.text("Get single depth")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {
                        request_single = true;
                        ArrayList<Double> single = getClosestDepth(loc.getLatitudeRads(), loc.getLongitudeRads(), grid_size); // exists: 63.322200, 10.169700
                        if(single.isEmpty())
                            NeptusLog.pub().info("grounding");
                        else
                            singles.add(single);
                        
                        NeptusLog.pub().info("SINGLE " + singles);
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else if(show_grounding || show_single)
        {
            popup.add(I18n.text("Hide single depth")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_single = false;
                    show_grounding = false;
                    request_single = false;
                    ground.clear();
                    singles.clear();
                }
            });
        }
    }

    /*private void addGetTransectMenu(JPopupMenu popup) {
        JMenuItem item = popup.add(I18n.text("Check transect"));
        item.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                try {
                    check_transect = true;
                } catch (Exception exc) {
                    // TODO: handle exception.
                }
            }
        });
    }*/

    private void addGetTransectMenu(JPopupMenu popup) {
        //JMenuItem item = popup.add(I18n.text("Show Transect Locations"));
        if(!show_single_transect)
        {
            popup.add(I18n.text("Check Transect")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    check_transect = true;
                }
            });
        } else
        {
            popup.add(I18n.text("Hide Transect")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_single_transect = false;
                    ground.clear();
                }
            });
        }
    }

    private void addGetSquareMenu(JPopupMenu popup, final LocationType loc) {
        //JMenuItem item = popup.add(I18n.text("Show Transect Locations"));
        if(!show_square)
        {
            popup.add(I18n.text("Get Square")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    loc.convertToAbsoluteLatLonDepth();
                    try {                    
                        NeptusLog.pub().info("QUERIED LAT " + loc.getLatitudeDegs() + " AND LON " + loc.getLongitudeDegs());
                        boolean dp = false;
                        square = getSquare(loc.getLatitudeRads(), loc.getLongitudeRads(), square_side, dp);
                        //NeptusLog.pub().info("LAT " + square.get(0) + " LON " + square.get(1) + " DEPTH " + square.get(2));
                        show_square = true;
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            popup.add(I18n.text("Hide Square")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_square = false;
                }
            });
        }
    }

    private void addGetWithinRadiusMenu(JPopupMenu popup, final LocationType loc) {
        if(!show_circle)
        {
            popup.add(I18n.text("Get Circle")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    loc.convertToAbsoluteLatLonDepth();
                    try {                    
                        NeptusLog.pub().info("QUERIED LAT " + loc.getLatitudeDegs() + " AND LON " + loc.getLongitudeDegs());
                        circle = getWithinRadius(loc.getLatitudeRads(), loc.getLongitudeRads(), circle_radius);
                        show_circle = true;
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            popup.add(I18n.text("Hide Circle")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_circle = false;
                }
            });
        }
    }

    private void addCurrentPlanMenu(JPopupMenu popup) {
        JMenuItem item = popup.add(I18n.text("Analyze current plan"));
        item.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                try {
                    for(int i=1; i<current_targets_lat.size()+1; i++)
                    {
                        plan_waypoint = getClosestDepth(current_targets_lat.get(i-1), current_targets_lon.get(i-1), grid_size);
                        plan_waypoints.add(plan_waypoint);
                        
                        if(current_targets_lat.size() == 1)
                        {
                            // Just one GoTo.
                            transect_start.setLocation(autonaut);
                            transect_end.setLatitudeRads(current_targets_lat.get(i-1));
                            transect_end.setLongitudeRads(current_targets_lon.get(i-1));
                        } else if(current_targets_lat.size() > 1)
                        {
                            transect_start.setLatitudeRads(current_targets_lat.get(i-1));
                            transect_start.setLongitudeRads(current_targets_lon.get(i-1));
                            transect_end.setLatitudeRads(current_targets_lat.get(i));
                            transect_end.setLongitudeRads(current_targets_lon.get(i));
                        }
                        ArrayList<ArrayList<Double>> transect = checkTransect(transect_start,transect_end);
                        transects.addAll(transect);
                    }
                } catch (Exception exc) {
                    // TODO: handle exception.
                }
            }
        });
    }

    private void addShowWaypointsMenu(JPopupMenu popup) {
        //JMenuItem item = popup.add(I18n.text("Show Plan Waypoints"));
        if(!show_waypoints)
        {
            popup.add(I18n.text("Show Plan Waypoints")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_waypoints = true;
                }
            });
        } else
        {
            popup.add(I18n.text("Hide Plan Waypoints")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_waypoints = false;
                }
            });
        }
    }

    private void addShowTransectMenu(JPopupMenu popup) {
        //JMenuItem item = popup.add(I18n.text("Show Plan Waypoints"));
        if(!show_transects)
        {
            popup.add(I18n.text("Show Plan Transects")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_transects = true;
                }
            });
        } else
        {
            popup.add(I18n.text("Hide Plan Transects")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_transects = false;
                }
            });
        }
    }

    private void addShowFeaturesMenu(JPopupMenu popup) {
        JMenu featuresMenu = new JMenu("Features");

        if(!show_buoys)
        {
            featuresMenu.add(new JMenuItem("Show Buoys")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {                    
                        manageFeature(buoys_layer, buoys);
                        show_buoys = true;
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            featuresMenu.add(new JMenuItem("Hide Buoys")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_buoys = false;
                }
            });
        }

        if(!show_beacons)
        {
            featuresMenu.add(new JMenuItem("Show Beacons")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {                    
                        manageFeature(beacons_layer, beacons);
                        show_beacons = true;
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            featuresMenu.add(new JMenuItem("Hide Beacons")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_beacons = false;
                }
            });
        }

        if(!show_dangers)
        {
            featuresMenu.add(new JMenuItem("Show Dangers")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {                    
                        manageFeature(dangers_layer, dangers);
                        show_dangers = true;
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            featuresMenu.add(new JMenuItem("Hide Dangers")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_dangers = false;
                }
            });
        }

        if(!show_regimes)
        {
            featuresMenu.add(new JMenuItem("Show Regimes")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {                    
                        manageFeature(regimes_layer, regimes);
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            featuresMenu.add(new JMenuItem("Hide Regimes")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_regimes = false;
                }
            });
        }

        if(!show_lights)
        {
            featuresMenu.add(new JMenuItem("Show Lights")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {                    
                        manageFeature(lights_layer, lights);
                        show_lights = true;
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            featuresMenu.add(new JMenuItem("Hide Lights")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_lights = false;
                }
            });
        }

        if(!show_navlines)
        {
            featuresMenu.add(new JMenuItem("Show Navigation Lines")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {                    
                        manageFeature(navlines_layer, navlines);
                        show_navlines = true;
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            featuresMenu.add(new JMenuItem("Hide Navigation Lines")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_navlines = false;
                }
            });
        }

        if(!show_obstructions)
        {
            featuresMenu.add(new JMenuItem("Show Obstructions")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {                    
                        manageFeature(obstructions_layer, obstructions);
                        show_obstructions = true;
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            featuresMenu.add(new JMenuItem("Hide Obstructions")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_obstructions = false;
                }
            });
        }

        if(!show_piers)
        {
            featuresMenu.add(new JMenuItem("Show Pier Extensions")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {                    
                        manageFeature(piers_layer, piers);
                        show_piers = true;
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            featuresMenu.add(new JMenuItem("Hide Pier Extensions")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_piers = false;
                }
            });
        }

        if(!show_stones)
        {
            featuresMenu.add(new JMenuItem("Show Stony Material")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {                    
                        manageFeature(stones_layer, stones);
                        show_stones = true;
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            featuresMenu.add(new JMenuItem("Hide Stony Material")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_stones = false;
                }
            });
        }

        if(!show_depth_cont)
        {
            //jc.setEditable(true);
            //jd.setEditable(true);
            featuresMenu.add(new JMenuItem("Show Depth Contours")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {

                    /*JTextField fside = new JTextField();
                    JTextField fdepth = new JTextField();

                    Object[] options = { "OK", "CANCEL" };

                    Object[] message = {
                        "Square side (km)", fside,
                        "Contour depth (m)", fdepth,
                    };
                    
                    int option = JOptionPane.showConfirmDialog(getConsole(), message, "Depth Contours Query", JOptionPane.OK_CANCEL_OPTION);
                    if (option == JOptionPane.OK_OPTION)
                    {
                        double side = Double.parseDouble(fside.getText());
                        double depth = Double.parseDouble(fdepth.getText());
                        if((side <= 0 || side > 20) && (depth > 0 && depth < 20))
                            JOptionPane.showOptionDialog(getConsole(), "Choose contour query in the range [0,20] km", "Warning",JOptionPane.DEFAULT_OPTION, JOptionPane.WARNING_MESSAGE,null, options, options[0]);
                        if((depth <= 0 || depth > 20) && (side > 0 && side < 20) )
                            JOptionPane.showOptionDialog(getConsole(), "Choose contour depth in the range [0,20] meters", "Warning",JOptionPane.DEFAULT_OPTION, JOptionPane.WARNING_MESSAGE,null, options, options[0]);
                        if((side <= 0 || side > 20) && (depth <= 0 || depth >20))
                            JOptionPane.showOptionDialog(getConsole(), "Choose contour query in the range [0,20] km"+"\n"+"Choose contour depth in the range [0,20] meters", "Warning",JOptionPane.DEFAULT_OPTION, JOptionPane.WARNING_MESSAGE,null, options, options[0]);
                        
                        if((depth > 0 && depth <= 20) && (side > 0 && side <= 20))
                        {
                            depth_contour.add(side);
                            depth_contour.add(depth);

                            try {
                                manageFeature(depth_cont_layer, depth_conts);
                                show_depth_cont = true;
                            } catch (Exception exc) {
                            }
                        }
                    }*/

                    /*//create a JOptionPane
                    Object[] options = new Object[] {"OK"};
                    JOptionPane jop = new JOptionPane("Please select square side (km) and depth (m) for depth contours",
                                                    JOptionPane.QUESTION_MESSAGE,
                                                    JOptionPane.DEFAULT_OPTION,
                                                    null,options, null);

                    //add combos to JOptionPane
                    jop.add(jc);
                    jop.add(jd);

                    //create a JDialog and add JOptionPane to it 
                    JDialog diag = new JDialog();
                    diag.getContentPane().add(jop);
                    diag.pack();
                    diag.setVisible(true);*/
                    
                    Object[] options = { "OK", "CANCEL" };
                    String radiusStr = JOptionPane.showInputDialog(getConsole(), I18n.text("Please enter square side (km) for depth contours"), radius_depare);
                    if (radiusStr == null)
                        return;
                    try {
                        radius_depare = Float.parseFloat(radiusStr);
                        if (radius_depare < 1 || radius_depare > 20)
                            JOptionPane.showOptionDialog(getConsole(), "Choose contour query in the range [0,20] km", "Warning",JOptionPane.DEFAULT_OPTION, JOptionPane.WARNING_MESSAGE,null, options, options[0]);
                        else
                        {
                            manageFeature(depth_cont_layer, depth_conts);
                            show_depth_cont = true;
                        }
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            featuresMenu.add(new JMenuItem("Hide Depth Contours")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_depth_cont = false;
                    depth_conts.clear();
                    depth_contour.clear();
                }
            });
        }

        if(!show_wrecks)
        {
            featuresMenu.add(new JMenuItem("Show Wrecks")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {
                        manageFeature(wrecks_layer, wrecks);
                        show_wrecks = true;
                    } catch (Exception exc) {
                        // TODO: handle exception.
                    }
                }
            });
        } else
        {
            featuresMenu.add(new JMenuItem("Hide Wrecks")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_wrecks = false;
                }
            });
        }
        
        popup.add(featuresMenu);
    }

    private void addShowSoundingsMenu(JPopupMenu popup) {
        //JMenuItem item = popup.add(I18n.text("Show Transect Locations"));
        if(!show_soundings)
        {
            popup.add(I18n.text("Show Soundings")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_soundings = true;
                }
            });
        } else
        {
            popup.add(I18n.text("Hide Soundings")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_soundings = false;
                }
            });
        }
    }


    private void addShowLegendMenu(JPopupMenu popup) {
        //JMenuItem item = popup.add(I18n.text("Show Transect Locations"));
        if(!show_legend)
        {
            popup.add(I18n.text("Show Color Legend")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_legend = true;
                }
            });
        } else
        {
            popup.add(I18n.text("Hide Color Legend")).addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    show_legend = false;
                }
            });
        }
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

    public void manageFeature(String name, ArrayList<ArrayList<Double>> fill) throws SQLException {
        ArrayList<Double> lats = new ArrayList<Double>();
        ArrayList<Double> lons = new ArrayList<Double>();

        if(name.equals(depth_cont_layer))
        {
            boolean dp = true;
            depth_conts = getSquare(mouse_click.getLatitudeRads(), mouse_click.getLongitudeRads(), radius_depare*1000, dp);
        } else if(name.equals(regimes_layer))
        {
            double lat=0.0,lon=0.0;
            String statem = "select Lat,Lon from " + name + " limit 1;";
            System.out.printf("%s\n", statem);
            synchronized (ser.conn) {
                Statement st = ser.conn.createStatement();
                ResultSet rs = st.executeQuery(statem);
                if (!rs.isBeforeFirst() ) {
                    System.out.println(" ----------- The query has produced no data! -------- ");
                }
                while(rs.next()) {
                    lat=rs.getDouble(1);
                    lon=rs.getDouble(2);
                    System.out.printf("%f, %f\n", Math.toDegrees(rs.getDouble(1)), Math.toDegrees(rs.getDouble(2)));
                }
                rs.close();
            }
            String statem_n = "select Lat,Lon from " + name + " order by abs(Lat-"+lat+") + abs(Lon-"+lon+") asc;";
            System.out.printf("%s\n", statem_n);
            synchronized (ser.conn) {
                Statement st = ser.conn.createStatement();
                ResultSet rs = st.executeQuery(statem_n);
                if (!rs.isBeforeFirst() ) {
                    System.out.println(" ----------- The query has produced no data! -------- ");
                }
                while(rs.next()) {
                    //LocationType vertex = new LocationType(Math.toDegrees(rs.getDouble(1)),Math.toDegrees(rs.getDouble(2)));
                    //regimes_vertices.add(vertex);
                    lats.add(rs.getDouble(1));
                    lons.add(rs.getDouble(2));
                    //System.out.printf("%f, %f\n", Math.toDegrees(rs.getDouble(1)), Math.toDegrees(rs.getDouble(2)));
                }
                rs.close();
            }
            fill.add(lats);
            fill.add(lons);
            buildPolygon();
        } else
        {
            String statem = "select Lat,Lon from " + name + ";";
            System.out.printf("%s\n", statem);
            synchronized (ser.conn) {
                Statement st = ser.conn.createStatement();
                ResultSet rs = st.executeQuery(statem);
                if (!rs.isBeforeFirst() ) {
                    System.out.println(" ----------- The query has produced no data! -------- ");
                }
                while(rs.next()) {
                    lats.add(rs.getDouble(1));
                    lons.add(rs.getDouble(2));
                    //System.out.printf("%f, %f\n", rs.getDouble(1), rs.getDouble(2));
                }
                rs.close();
            }

            fill.add(lats);
            fill.add(lons); 
        }
    }

    public void buildPolygon() {

        int reg_locs = regimes.get(0).size();
        double max_lat = Collections.max(regimes.get(0));
        double min_lat = Collections.min(regimes.get(0));
        ArrayList<Integer> max_lat_indexes = new ArrayList<Integer>();
        ArrayList<Integer> min_lat_indexes = new ArrayList<Integer>();
        ArrayList<Double> lons1 = new ArrayList<Double>();
        ArrayList<Double> lons2 = new ArrayList<Double>();

        for(int i=0; i<reg_locs;i++)
        {
            if(regimes.get(0).get(i)==max_lat)
                max_lat_indexes.add(i);
            else if(regimes.get(0).get(i)==min_lat)
                min_lat_indexes.add(i);
        }

        for(int j=0; j<max_lat_indexes.size(); j++) {
            int index = max_lat_indexes.get(j);
            lons1.add(regimes.get(1).get(index));
        }
        for(int k=0; k<min_lat_indexes.size(); k++) {
            int index = min_lat_indexes.get(k);
            lons2.add(regimes.get(1).get(index));
        }

        double max_lon_for_max_lat = Collections.max(lons1);
        double min_lon_for_max_lat = Collections.min(lons1);
        double max_lon_for_min_lat = Collections.max(lons2);
        double min_lon_for_min_lat = Collections.min(lons2);

        LocationType top_left = new LocationType(Math.toDegrees(max_lat),Math.toDegrees(min_lon_for_max_lat));
        LocationType top_right = new LocationType(Math.toDegrees(max_lat),Math.toDegrees(max_lon_for_max_lat));
        LocationType bottom_left = new LocationType(Math.toDegrees(min_lat),Math.toDegrees(min_lon_for_min_lat));
        LocationType bottom_right = new LocationType(Math.toDegrees(min_lat),Math.toDegrees(max_lon_for_min_lat));

        regimes_vertices.add(top_left);
        regimes_vertices.add(top_right);
        regimes_vertices.add(bottom_left);
        regimes_vertices.add(bottom_right);

        show_regimes = true;
    }

    public ArrayList<Double> getClosestDepth(double lat, double lon, double grid_size) throws SQLException {
        ArrayList<Double> lats;
        ArrayList<Double> lons;
        ArrayList<Double> depths;
        ArrayList<Double> ret = new ArrayList<Double>();
        double[] bear_range;
        double[] ranges = {0.0,0.0,0.0,0.0};
        boolean gr = false;
        ArrayList<Double> ground_location = new ArrayList<Double>();
        
        LocationType queried = new LocationType(lat,lon);

        String statem = "select Depth from 'depthmapRad' where Lat = " + lat + " and Lon = " + lon + ";";
        double depth = 0.0;
        synchronized (ser.conn) {
            Statement st = ser.conn.createStatement();
            ResultSet rs = st.executeQuery(statem);
            if (!rs.isBeforeFirst() ) {
                System.out.println(" ----------- The query has produced no data! -------- ");
            }
            while(rs.next()) {
                depth = rs.getDouble(1);
                ret.add(lat);
                ret.add(lon);
                ret.add(depth);
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
            NeptusLog.pub().info("FOUR CLOSEST " + four_points +  "\n");

            for(int j=0; j<lats.size(); j++)
            {
                if(!gr && lats.get(j) == 0.0)
                    gr = true;
            }

            if(!gr && lats.size() == 4)
            {
                //show_single_transect = true;
                for(int i=0; i<lats.size(); i++)
                {
                    LocationType close = new LocationType(Math.toDegrees(lats.get(i)),Math.toDegrees(lons.get(i)));
                    bear_range = CoordinateUtil.getNEBearingDegreesAndRange(queried,close);
                    ranges[i] = bear_range[1];
                    //NeptusLog.pub().info("BEARING " + bear_range[0] + " AND RANGE "  + bear_range[1]);
                }
                double[] min = getMin(ranges);
                //NeptusLog.pub().info("MIN RANGE " + min[0] + " ITS INDEX " + min[1]);
                double lat_add = lats.get((int)min[1]);
                double lon_add = lons.get((int)min[1]);
                double depth_add = depths.get((int)min[1]);
                ret.add(lat_add);
                ret.add(lon_add);
                ret.add(depth_add);
                if(request_single)
                    show_single = true;
            } else if(gr || lats.size() < 4)
            {
                NeptusLog.pub().warn("POSSIBLE GROUNDING!");
                ground_location.add(lat);
                ground_location.add(lon);
                ground.add(ground_location);
                show_grounding = true;
            }
            
            //NeptusLog.pub().info("CLOSEST LOCATION LAT " + lat_add + " LON " + lon_add + " DEPTH " +depth_add);
        }
        //NeptusLog.pub().info("CLOSEST DEPTH " + depth);
        return ret;
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
        double [] lat_minus_disp = CoordinateUtil.WGS84displace(Math.toDegrees(Lat), Math.toDegrees(Lon), 0.0, -disp, 0.0, 0.0);
        double lat_minus_displaced = Math.toRadians(lat_minus_disp[0]);
        //System.out.printf("LAT_D %f, LON_D %f, DEPTH_D %f\n", lat_minus_disp[0], lat_minus_disp[1], lat_minus_disp[2]);

        //double lat_plus_displaced = Lat_rad;
        //WGS84::displace(disp,0.0,&lat_plus_displaced, &Lon_rad);
        double [] lat_plus_disp = CoordinateUtil.WGS84displace(Math.toDegrees(Lat), Math.toDegrees(Lon), 0.0, disp, 0.0, 0.0);
        double lat_plus_displaced = Math.toRadians(lat_plus_disp[0]);
        //System.out.printf("LAT_D %f, LON_D %f, DEPTH_D %f\n", lat_plus_disp[0], lat_plus_disp[1], lat_plus_disp[2]);

        //double lon_minus_displaced = Lon_rad;
        //WGS84::displace(0.0,-disp,&Lat_rad, &lon_minus_displaced);
        double [] lon_minus_disp = CoordinateUtil.WGS84displace(Math.toDegrees(Lat), Math.toDegrees(Lon), 0.0, 0.0, -disp, 0.0);
        double lon_minus_displaced = Math.toRadians(lon_minus_disp[1]);
        //System.out.printf("LAT_D %f, LON_D %f, DEPTH_D %f\n", lon_minus_disp[0], lon_minus_disp[1], lon_minus_disp[2]);

        //double lon_plus_displaced = Lon_rad;
        //WGS84::displace(0.0,disp,&Lat_rad, &lon_plus_displaced);
        double [] lon_plus_disp = CoordinateUtil.WGS84displace(Math.toDegrees(Lat), Math.toDegrees(Lon), 0.0, 0.0, disp, 0.0);
        double lon_plus_displaced = Math.toRadians(lon_plus_disp[1]);
        //System.out.printf("LAT_D %f, LON_D %f, DEPTH_D %f\n", lon_plus_disp[0], lon_plus_disp[1], lon_plus_disp[2]);

        String c_stmt = "select min(Lat+Lon), Lat, Lon, Depth from (select Lat, Lon, Depth from depthmapRad where Lat between " + lat_minus_displaced + " and " + lat_plus_displaced + " and Lon between " + lon_minus_displaced + " and " + lon_plus_displaced + ") where Lat >= " + Lat + " and Lon >= " + Lon +
        " union select max(Lat+Lon), Lat, Lon, Depth from (select Lat, Lon, Depth from depthmapRad where Lat between " + lat_minus_displaced + " and " + lat_plus_displaced + " and Lon between " + lon_minus_displaced + " and " + lon_plus_displaced + ") where Lat <= " + Lat + " and Lon <= " + Lon + 
        " union select min(Lat-Lon), Lat, Lon, Depth from (select Lat, Lon, Depth from depthmapRad where Lat between " + lat_minus_displaced + " and " + lat_plus_displaced + " and Lon between " + lon_minus_displaced + " and " + lon_plus_displaced + ") where Lat >= " + Lat + " and Lon <= " + Lon + 
        " union select max(Lat-Lon), Lat, Lon, Depth from (select Lat, Lon, Depth from depthmapRad where Lat between " + lat_minus_displaced + " and " + lat_plus_displaced + " and Lon between " + lon_minus_displaced + " and " + lon_plus_displaced + ") where Lat <= " + Lat + " and Lon >= " + Lon + ";";

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

    public ArrayList<ArrayList<Double>> getSquare(double Lat, double Lon, double half_size, boolean is_depare) throws SQLException {

        ArrayList<ArrayList<Double>> all = new ArrayList<>();
        ArrayList<Double> lats = new ArrayList<>();
        ArrayList<Double> lons = new ArrayList<>();
        ArrayList<Double> drvals1 = new ArrayList<>();
        ArrayList<Double> drvals2 = new ArrayList<>();
        ArrayList<Double> depths = new ArrayList<>();
        String c_stmt = "";
        
        double [] lat_minus_disp = CoordinateUtil.WGS84displace(Math.toDegrees(Lat), Math.toDegrees(Lon), 0.0, -half_size, 0.0, 0.0);
        double lat_minus_displaced = Math.toRadians(lat_minus_disp[0]);
        //System.out.printf("LAT_D_S %f, LON_D_S %f, DEPTH_D_S %f\n", lat_minus_disp[0], lat_minus_disp[1], lat_minus_disp[2]);

        double [] lat_plus_disp = CoordinateUtil.WGS84displace(Math.toDegrees(Lat), Math.toDegrees(Lon), 0.0, half_size, 0.0, 0.0);
        double lat_plus_displaced = Math.toRadians(lat_plus_disp[0]);
        //System.out.printf("LAT_D_S %f, LON_D_S %f, DEPTH_D_S %f\n", lat_plus_disp[0], lat_plus_disp[1], lat_plus_disp[2]);

        double [] lon_minus_disp = CoordinateUtil.WGS84displace(Math.toDegrees(Lat), Math.toDegrees(Lon), 0.0, 0.0, -half_size, 0.0);
        double lon_minus_displaced = Math.toRadians(lon_minus_disp[1]);
        //System.out.printf("LAT_D_S %f, LON_D_S %f, DEPTH_D_S %f\n", lon_minus_disp[0], lon_minus_disp[1], lon_minus_disp[2]);

        double [] lon_plus_disp = CoordinateUtil.WGS84displace(Math.toDegrees(Lat), Math.toDegrees(Lon), 0.0, 0.0, half_size, 0.0);
        double lon_plus_displaced = Math.toRadians(lon_plus_disp[1]);
        //System.out.printf("LAT_D_S %f, LON_D_S %f, DEPTH_D_S %f\n", lon_plus_disp[0], lon_plus_disp[1], lon_plus_disp[2]);

        if(!is_depare)
        {
            c_stmt = c_stmt + "select Lat, Lon, Depth from 'depthmapRad' where Lat between " + lat_minus_displaced + " and " + lat_plus_displaced + " and Lon between " + lon_minus_displaced + " and " + lon_plus_displaced + ";";
            System.out.printf("%s\n", c_stmt);

            synchronized (ser.conn) {
                Statement st = ser.conn.createStatement();
                ResultSet rs = st.executeQuery(c_stmt);

                if (!rs.isBeforeFirst()) {
                    System.out.println(" ----------- The query has produced no data! -------- ");
                }

                while(rs.next()) {
                    lats.add(rs.getDouble(1));
                    lons.add(rs.getDouble(2));
                    depths.add(rs.getDouble(3));
                    //System.out.printf("%f, %f, %f\n", Math.toDegrees(rs.getDouble(1)), Math.toDegrees(rs.getDouble(2)), rs.getDouble(3));
                }
                rs.close();
            }
            all.add(lats);
            all.add(lons);
            all.add(depths);
        }
        else
        {  
            c_stmt = c_stmt + "select * from 'DEPARE' where Lat between " + lat_minus_displaced + " and " + lat_plus_displaced + " and Lon between " + lon_minus_displaced + " and " + lon_plus_displaced + ";"; //DRVAL1 > 1.0 AND
            System.out.printf("%s\n", c_stmt);

            synchronized (ser.conn) {
                Statement st = ser.conn.createStatement();
                ResultSet rs = st.executeQuery(c_stmt);

                if (!rs.isBeforeFirst()) {
                    System.out.println(" ----------- The query has produced no data! -------- ");
                }

                while(rs.next()) {
                    //System.out.printf("%f, %f, %f, %f\n", Math.toDegrees(rs.getDouble(1)), Math.toDegrees(rs.getDouble(2)), rs.getDouble(3), rs.getDouble(4));
                    lats.add(rs.getDouble(2));
                    lons.add(rs.getDouble(1));
                    drvals1.add(rs.getDouble(3));
                    drvals2.add(rs.getDouble(4));
                    
                }
                rs.close();
            }
            all.add(lats);
            all.add(lons);
            all.add(drvals1);
            all.add(drvals2);
        }
        return all;
    }

    public ArrayList<ArrayList<Double>> getWithinRadius(double Lat, double Lon, double radius) throws SQLException {
        ArrayList<Double> lats, lats_close = new ArrayList<>();
        ArrayList<Double> lons, lons_close = new ArrayList<>();
        ArrayList<Double> depths, depths_close = new ArrayList<>();
        ArrayList<ArrayList<Double>> ret = new ArrayList<ArrayList<Double>>(); // contains all the points (lat,lon,depth) within the radius.
        double[] bear_range;
        
        LocationType queried = new LocationType(Math.toDegrees(Lat),Math.toDegrees(Lon));

        boolean dp = false;
        ArrayList<ArrayList<Double>> square = getSquare(Lat,Lon,radius,dp);
        lats = square.get(0);
        lons = square.get(1);
        depths = square.get(2);

        for(int i=0; i<lats.size(); i++)
        {
            LocationType close = new LocationType(Math.toDegrees(lats.get(i)),Math.toDegrees(lons.get(i)));
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

    @Subscribe
    public void on(PlanSpecification msg) {
        NeptusLog.pub().info("PLAN SPECIFICATION RECEIVED");
        planSpec = msg;

        // Clear current targets.
        current_targets_lat.clear();
        current_targets_lon.clear();
        // Clear current painted waypoints.
        plan_waypoints.clear();
        transects.clear();

        //newPlan = msg.getAsString("plan_id");

        if (planSpec != null) {
            for (PlanManeuver planMan : planSpec.getManeuvers()) {
                Maneuver man = planMan.getData();
                double lat = man.getAsNumber("lat").doubleValue();
                double lon = man.getAsNumber("lon").doubleValue();
                current_targets_lat.add(lat);
                current_targets_lon.add(lon);
                System.out.printf("PLANNED LAT %f, LON %f\n", lat, lon);
            }
        }
    }

    @Subscribe
    public void on(GpsFix msg) {
        double aut_lat = msg.getLat();
        double aut_lon = msg.getLon();
        autonaut.setLatitudeRads(aut_lat);
        autonaut.setLongitudeRads(aut_lon);
        //NeptusLog.pub().info("AUTONAUT LAT " + Math.toDegrees(aut_lat) + " AND LON " + Math.toDegrees(aut_lon));
    }
    
    @Subscribe
    public void on(PlanDB msg) {
    }

    public ArrayList<ArrayList<Double>> checkTransect(LocationType start, LocationType end) throws SQLException {
        double[] bear_range = CoordinateUtil.getNEBearingDegreesAndRange(start,end);
        double steps = bear_range[1]/grid_size;
        double startLat = start.getLatitudeRads();
        double endLat = end.getLatitudeRads();
        double startLon = start.getLongitudeRads();
        double endLon = end.getLongitudeRads();
        double stepLat = (endLat-startLat)/steps;
        double stepLon = (endLon-startLon)/steps;
        ArrayList<ArrayList<Double>> sampled_locs = new ArrayList<ArrayList<Double>>();

        for(int step=0; step<steps; step++)
        {
            ArrayList<Double> closest_loc = getClosestDepth(startLat+step*stepLat, startLon+step*stepLon, grid_size);
            if(closest_loc.isEmpty())
                NeptusLog.pub().info("grounding");
            else
                sampled_locs.add(closest_loc);
        }
        return sampled_locs;
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

    /*@Subscribe
    public void on(PlanControlState msg) {
        if (msg.getSourceName().equals(getConsole().getMainSystem())) {
            // if the vehicle is currently executing a plan we ask for that plan
            // and then identify what maneuver is being executed
            if (msg.getAsNumber("state").longValue() == STATE.EXECUTING.value()) {

                NeptusLog.pub().info("EXECUTING");

                if(!msg.getAsString("plan_id").equals(currentPlan))
                    samePlan = false;

                currentPlan = msg.getAsString("plan_id");
                currentManeuver = msg.getAsString("man_id");

                if (planSpec != null && samePlan) {
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
    }*/

    /**
     * Always returns true
     */
    @Override
    public boolean isExclusive() {
        return true;
    }

    public Color chooseColor(double depth){
        Color color = new Color(0);
        if(depth < -1000)
            color = new Color(43, 15, 249);
        else if(depth >= -1000 && depth < -500)
            color = new Color(71, 13, 197);
        else if(depth >= -500 && depth < -250)
            color = new Color(98, 12, 145);
        else if(depth >= -250 && depth < -100)
            color = new Color(135, 9, 76);
        else if(depth >= -100 && depth < -50)
            color = new Color(154, 8, 42);
        else if(depth >= -50 && depth < -20)
            color = new Color(163, 8, 24);
        else if(depth >= -20 && depth < 0)
            color = new Color(172, 7, 7);

        return color;
    }

    public Color chooseContourColor(double drval2){
        Color color = new Color(0);
        if(drval2 == 0.5)
            color = new Color(0,0,0);
        else if(drval2 == 5.0)
            color = new Color(0,0,128);
        else if(drval2 == 10.0)
            color = new Color(0,0,205);
        else if(drval2 == 15.0)
            color = new Color(0,0,255);
        else if(drval2 == 20.0)
            color = new Color(102,102,255);

        return color;
    }

    /**
     * Paints filled circles on the current target and drop positions.
     */
    //@Override
    public void paint(Graphics2D g, StateRenderer2D renderer) {
        super.paint(g, renderer);

        if(show_legend)
            paintColorbars(g, renderer);

        // If the land position has not been set, there is nothing to paint
        if (!show_grounding && !show_waypoints && !show_transects && !show_single_transect && !show_single && !show_square && !show_circle && !show_buoys && !show_beacons && !show_dangers
            && !show_depth_cont && !show_lights && !show_navlines && !show_obstructions && !show_piers && !show_regimes && !show_stones && !show_wrecks)
            return;
        
        if(show_waypoints && !show_transects)
        {
            int wp_num = plan_waypoints.size();
        
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                ArrayList<Double> current_waypoint = plan_waypoints.get(i);
                double lat = current_waypoint.get(0);
                double lon = current_waypoint.get(1);
                double depth = -current_waypoint.get(2);
                String depth_str = Double.toString((double)Math.round(depth * 100d) / 100d);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                // Draws the "arrow"
                //g.setColor(Color.green);
                //g.fillPolygon(poly);

                // Should be done according to colormap, not like this..
                Color col = chooseColor(depth);
                clone.setColor(col);

                clone.fill(new Ellipse2D.Double(0, 0, 10, 10));
                if(show_soundings)
                    clone.drawString(I18n.text(depth_str), 10, 0);
            }
        } else if(show_transects)
        {
            int wp_num = transects.size();
        
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                ArrayList<Double> current_waypoint = transects.get(i);
                double lat = current_waypoint.get(0);
                double lon = current_waypoint.get(1);
                double depth = -current_waypoint.get(2);
                String depth_str = Double.toString((double)Math.round(depth * 100d) / 100d);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                // Should be done according to colormap, not like this..
                Color col = chooseColor(depth);
                clone.setColor(col);

                clone.fill(new Ellipse2D.Double(0, 0, 10, 10));
                if(show_soundings)
                    clone.drawString(I18n.text(depth_str), 10, 0);
            }
        }

        if(show_single)
        {
            int wp_num = singles.size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = singles.get(i).get(0);
                double lon = singles.get(i).get(1);
                double depth = -singles.get(i).get(2);
                String depth_str = Double.toString((double)Math.round(depth * 100d) / 100d);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                // Should be done according to colormap, not like this..
                Color col = chooseColor(depth);
                clone.setColor(col);

                clone.fill(new Ellipse2D.Double(0, 0, 10, 10));
                if(show_soundings)
                    clone.drawString(I18n.text(depth_str), 10, 0);
            }
        }

        if(show_single_transect)
        {
            int wp_num = single_transect.size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                ArrayList<Double> current_loc = single_transect.get(i);
                double lat = current_loc.get(0);
                double lon = current_loc.get(1);
                double depth = -current_loc.get(2);
                String depth_str = Double.toString((double)Math.round(depth * 100d) / 100d);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                // Should be done according to colormap, not like this..
                Color col = chooseColor(depth);
                clone.setColor(col);

                clone.fill(new Ellipse2D.Double(0, 0, 10, 10));
                if(show_soundings)
                    clone.drawString(I18n.text(depth_str), 10, 0);
            }
        }

        if(show_square)
        {
            int wp_num = square.get(0).size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = square.get(0).get(i);
                double lon = square.get(1).get(i);
                double depth = -square.get(2).get(i);
                String depth_str = Double.toString((double)Math.round(depth * 100d) / 100d);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                // Should be done according to colormap, not like this..
                Color col = chooseColor(depth);
                clone.setColor(col);

                clone.fill(new Ellipse2D.Double(0, 0, 10, 10));
                if(show_soundings)
                    clone.drawString(I18n.text(depth_str), 10, 0);
            }
        }

        if(show_circle)
        {
            int wp_num = circle.get(0).size();

            //NeptusLog.pub().info("SIIIIIZEEEE " + wp_num);
        
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = circle.get(0).get(i);
                double lon = circle.get(1).get(i);
                double depth = -circle.get(2).get(i);
                String depth_str = Double.toString((double)Math.round(depth * 100d) / 100d);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                // Should be done according to colormap, not like this..
                Color col = chooseColor(depth);
                clone.setColor(col);

                clone.fill(new Ellipse2D.Double(0, 0, 10, 10));
                if(show_soundings)
                    clone.drawString(I18n.text(depth_str), 10, 0);
            }
        }

        if(show_buoys)
        {
            int wp_num = buoys.get(0).size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = buoys.get(0).get(i);
                double lon = buoys.get(1).get(i);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                //clone.fill(new Rectangle2D.Double(0, 0, 10, 10));
                //clone.drawString(I18n.text("buoy"), 10, 0);
                clone.drawImage(images.get(0), 0, 0, null);
            }
        }

        if(show_beacons)
        {
            int wp_num = beacons.get(0).size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = beacons.get(0).get(i);
                double lon = beacons.get(1).get(i);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                clone.drawImage(images.get(1), 0, 0, null);
            }
        }

        if(show_dangers)
        {
            int wp_num = dangers.get(0).size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = dangers.get(0).get(i);
                double lon = dangers.get(1).get(i);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                clone.drawImage(images.get(8), 0, 0, null);
            }
        }

        if(show_depth_cont)
        {
            int wp_num = depth_conts.get(0).size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = depth_conts.get(0).get(i);
                double lon = depth_conts.get(1).get(i);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                Color col = chooseContourColor(depth_conts.get(3).get(i));
                clone.setColor(col);
                clone.fill(new Rectangle2D.Double(0, 0, 5, 5));
                //clone.drawString(I18n.text("dc"), 10, 0);
            }
        }

        if(show_lights)
        {
            int wp_num = lights.get(0).size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = lights.get(0).get(i);
                double lon = lights.get(1).get(i);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                clone.drawImage(images.get(2), 0, 0, null);
            }
        }

        if(show_navlines)
        {
            int wp_num = navlines.get(0).size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = navlines.get(0).get(i);
                double lon = navlines.get(1).get(i);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                clone.fill(new Rectangle2D.Double(0, 0, 10, 10));
                clone.drawString(I18n.text("n"), 10, 0);
            }
        }

        if(show_obstructions)
        {
            int wp_num = obstructions.get(0).size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = obstructions.get(0).get(i);
                double lon = obstructions.get(1).get(i);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                clone.drawImage(images.get(3), 0, 0, null);
            }
        }

        if(show_piers)
        {
            int wp_num = piers.get(0).size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = piers.get(0).get(i);
                double lon = piers.get(1).get(i);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                clone.drawImage(images.get(4), 0, 0, null);
            }
        }

        if(show_regimes)
        {

            /*Graphics2D clone_points = (Graphics2D) g.create();

            int num = regimes.get(0).size();

            for(int i=0; i<num; i++)
            {
                double lat = regimes.get(0).get(i);
                double lon = regimes.get(1).get(i);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone_points.translate(pt.getX(), pt.getY());

                clone_points.fill(new Rectangle2D.Double(0, 0, 10, 10));
            }*/

            Graphics2D clone = (Graphics2D) g.create();

            int wp_num = regimes_vertices.size();
            int[] EXEZNE_arrX = new int[wp_num];
            int[] EXEZNE_arrY = new int[wp_num];

            for(int i=0; i<wp_num; i++)
            {
                LocationType location = regimes_vertices.get(i);
                Point2D pt = renderer.getScreenPosition(location);
                //clone.translate(pt.getX(), pt.getY());
                EXEZNE_arrX[i] = (int) Math.round(pt.getX());
                EXEZNE_arrY[i] = (int) Math.round(pt.getY());


                //poly.addPoint((int) Math.round(pt.getX()), (int) Math.round(pt.getY()));
                //poly.ypoints[i] = (int) Math.round(pt.getY());

                //clone.fill(new Rectangle2D.Double(0, 0, 10, 10));
                //clone.drawString(I18n.text("r"), 10, 0);
            }

            Polygon poly = new Polygon(EXEZNE_arrX, EXEZNE_arrY, wp_num);

            clone.setColor(new Color(255,0,0,127)); // 50% transparency
            clone.fillPolygon(poly);
        }

        if(show_stones)
        {
            int wp_num = stones.get(0).size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = stones.get(0).get(i);
                double lon = stones.get(1).get(i);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                clone.drawImage(images.get(5), 0, 0, null);
            }
        }

        if(show_wrecks)
        {
            int wp_num = wrecks.get(0).size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = wrecks.get(0).get(i);
                double lon = wrecks.get(1).get(i);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());

                clone.drawImage(images.get(6), 0, 0, null);
            }
        }

        if(show_grounding)
        {
            int wp_num = ground.size();
            for(int i=0; i<wp_num; i++)
            {
                Graphics2D clone = (Graphics2D) g.create();
                double lat = ground.get(i).get(0);
                double lon = ground.get(i).get(1);
                LocationType location = new LocationType(Math.toDegrees(lat), Math.toDegrees(lon));
                Point2D pt = renderer.getScreenPosition(location);
                clone.translate(pt.getX(), pt.getY());
                clone.drawImage(images.get(7), 0, 0, null);
            }
        }

    }

    private void paintColorbars(Graphics2D go, StateRenderer2D renderer) {
        int offsetHeight = 80;
        int offsetWidth = 15;
        int offsetHeight_d = 180;
        int offsetWidth_d = 15;

        Graphics2D gl = (Graphics2D) go.create();
        gl.translate(offsetWidth, offsetHeight);

        ArrayList<Color> colors_depths = new ArrayList<Color>();
        colors_depths.add(new Color(43, 15, 249));
        colors_depths.add(new Color(71, 13, 197));
        colors_depths.add(new Color(98, 12, 145));
        colors_depths.add(new Color(135, 9, 76));
        colors_depths.add(new Color(154, 8, 42));
        colors_depths.add(new Color(163, 8, 24));
        colors_depths.add(new Color(172, 7, 7));
        String[] colors_sound_depths = {"< -1000", "[-1000,-500]", "[-500,-250]", "[-250,-100]", "[-100,-50]","[-50,-20]", "[-20,0]"};
        for (int i = 0; i < colors_depths.size(); i++) {
            gl.setColor(colors_depths.get(i));
            gl.fillRect(20, 20 + 150 / colors_depths.size() * i, 20, 150 / colors_depths.size());
            if(i==0)
            {
                gl.drawString(I18n.text("Soundings"), 5, 5 + 150 / colors_depths.size() * i);
                gl.drawString(colors_sound_depths[i], 50, 35 + 150 / colors_depths.size() * i);
            } else
                gl.drawString(colors_sound_depths[i], 50, 35 + 150 / colors_depths.size() * i);
            
        }
        gl.dispose();

        if(show_depth_cont)
        {
            ArrayList<Color> colors_cont = new ArrayList<Color>();
            colors_cont.add(new Color(0,0,0));
            colors_cont.add(new Color(0,0,128));
            colors_cont.add(new Color(0,0,205));
            colors_cont.add(new Color(0,0,255));
            colors_cont.add(new Color(102,102,255));
            String[] colors_cont_depths = {"-0.5", "-5.0", "-10.0", "-15.0","-20.0"};

            Graphics2D gl_ = (Graphics2D) go.create();
            gl_.translate(offsetWidth_d, offsetHeight_d);

            for (int i = 0; i < colors_cont.size(); i++) {
                gl_.setColor(colors_cont.get(i));
                gl_.fillRect(20, 150 + 150 / colors_cont.size() * i, 20, 150 / colors_cont.size());
                //gl_.fillRect(frame.getWidth() / colors.length * i, 0, frame.getWidth()
                //            / colors.length, frame.getHeight());
                if(i==0)
                {
                    gl_.drawString(I18n.text("Contour"), 5, 140 + 150 / colors_cont.size() * i);
                    gl_.drawString(colors_cont_depths[i], 50, 170 + 150 / colors_cont.size() * i);
                } else
                    gl_.drawString(colors_cont_depths[i], 50, 170 + 150 / colors_cont.size() * i);
                
            }
            gl_.dispose();
        }
    }

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