/*
 * Copyright (c) 2004-2013 Universidade do Porto - Faculdade de Engenharia
 * Laboratório de Sistemas e Tecnologia Subaquática (LSTS)
 * All rights reserved.
 * Rua Dr. Roberto Frias s/n, sala I203, 4200-465 Porto, Portugal
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
 * European Union Public Licence - EUPL v.1.1 Usage
 * Alternatively, this file may be used under the terms of the EUPL,
 * Version 1.1 only (the "Licence"), appearing in the file LICENCE.md
 * included in the packaging of this file. You may not use this work
 * except in compliance with the Licence. Unless required by applicable
 * law or agreed to in writing, software distributed under the Licence is
 * distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF
 * ANY KIND, either express or implied. See the Licence for the specific
 * language governing permissions and limitations at
 * https://www.lsts.pt/neptus/licence.
 *
 * For more information please see <http://lsts.fe.up.pt/neptus>.
 *
 * Author: José Pinto
 * Jun 25, 2012
 */
package pt.up.fe.dceg.neptus.plugins.planning;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import pt.up.fe.dceg.neptus.console.ConsoleLayout;
import pt.up.fe.dceg.neptus.i18n.I18n;
import pt.up.fe.dceg.neptus.imc.IMCDefinition;
import pt.up.fe.dceg.neptus.imc.IMCMessage;
import pt.up.fe.dceg.neptus.imc.PathControlState;
import pt.up.fe.dceg.neptus.messages.MessageFilter;
import pt.up.fe.dceg.neptus.messages.listener.MessageInfo;
import pt.up.fe.dceg.neptus.messages.listener.MessageListener;
import pt.up.fe.dceg.neptus.plugins.PluginDescription;
import pt.up.fe.dceg.neptus.plugins.SimpleSubPanel;
import pt.up.fe.dceg.neptus.renderer2d.Renderer2DPainter;
import pt.up.fe.dceg.neptus.renderer2d.StateRenderer2D;
import pt.up.fe.dceg.neptus.types.coord.LocationType;
import pt.up.fe.dceg.neptus.util.comm.manager.imc.ImcMsgManager;
import pt.up.fe.dceg.neptus.util.comm.manager.imc.ImcSystem;
import pt.up.fe.dceg.neptus.util.comm.manager.imc.ImcSystemsHolder;

/**
 * @author zp
 *
 */
@PluginDescription(name="PathControlLayer")
public class PathControlLayer extends SimpleSubPanel implements Renderer2DPainter, MessageListener<MessageInfo, IMCMessage> {


    private static final long serialVersionUID = 1L;
    protected final int pcontrol_id = IMCDefinition.getInstance().getMessageId("PathControlState");
    
    protected Map<Integer, PathControlState> lastMsgs = Collections.synchronizedMap(new LinkedHashMap<Integer, PathControlState>());

    /**
     * @param console
     */
    public PathControlLayer(ConsoleLayout console) {
        super(console);
    }
    
    @Override
    public void initSubPanel() {
        ImcMsgManager.getManager().addListener(this, new MessageFilter<MessageInfo, IMCMessage>() {            
            @Override
            public boolean isMessageToListen(MessageInfo info, IMCMessage msg) {
                return msg.getMgid() == pcontrol_id;
            }
        });
    }
    
    @Override
    public void cleanSubPanel() {
        ImcMsgManager.getManager().removeListener(this);
    }


    @Override
    public void paint(Graphics2D g, StateRenderer2D renderer) {
        
        for (PathControlState pcs : lastMsgs.values()) {
            
            
                LocationType dest = new LocationType(Math.toDegrees(pcs.getEndLat()), Math.toDegrees(pcs.getEndLon()));
                ImcSystem system = ImcSystemsHolder.lookupSystem(pcs.getSrc());
                
                Point2D pt = renderer.getScreenPosition(dest);
                
                g.draw(new Ellipse2D.Double(pt.getX()-5, pt.getY()-5, 10, 10));
                
                if (system != null) {
                    LocationType src = system.getLocation();
                    Point2D ptSrc = renderer.getScreenPosition(src);
                    g.draw(new Line2D.Double(ptSrc, pt));
                }
           // }
        }
    }

    @Override
    public void onMessage(MessageInfo arg0, IMCMessage msg) {
        if (msg.getMgid() == pcontrol_id) {
            try {
                lastMsgs.put(msg.getSrc(), PathControlState.clone(msg));
            }
            catch (Exception e) {
                e.printStackTrace();
            }
        }       
    }        
}
