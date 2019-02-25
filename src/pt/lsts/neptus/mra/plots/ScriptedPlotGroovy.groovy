/*
 * Copyright (c) 2004-2019 Universidade do Porto - Faculdade de Engenharia
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
 * Modified European Union Public Licence - EUPL v.1.1 Usage
 * Alternatively, this file may be used under the terms of the Modified EUPL,
 * Version 1.1 only (the "Licence"), appearing in the file LICENSE.md
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
 * Author: keila
 * Feb 8, 2019
 */
package pt.lsts.neptus.mra.plots

import org.jfree.data.time.Millisecond
import org.jfree.data.time.TimeSeries
import org.jfree.data.time.TimeSeriesDataItem
import org.jfree.data.xy.XYDataItem
import org.jfree.data.xy.XYSeries

/**
 * @author keila
 *
 */
class ScriptedPlotGroovy  {
    
    static ScriptedPlot scripedPlot = null
    
    static void configPlot(ScriptedPlot p) {
        scripedPlot = p
    }
    
    static def value = { msgDotField ->
    	if(scripedPlot!= null)
    		return scripedPlot.getTimeSeriesFor(msgDotField);
    }

	static void addTimeSeries(LinkedHashMap<String,String> queries) {
        queries.each {
            scripedPlot.addTimeSeries(it.key, it.value)
        }
	}
    
    static void addTimeSeries(String... queries) {
       queries.each { 
           scripedPlot.addTimeSeries(it, it)
       }
    }
    
    static LinkedHashMap<String,TimeSeries> addTimeSeries(TimeSeries ts) {
        scripedPlot.addTimeSeries(ts)
    }
 
    static void addQuery(String id,String query) {
        scripedPlot.addQuery(id,query)
    }
    
    static LinkedHashMap<String,TimeSeries> getTimeSeries(List<String> fields) {
        fields.forEach{
            scripedPlot.addTimeSeries(it)
        }
    }
    
    static public TimeSeries apply(String id,TimeSeries ts, Object function) {
        TimeSeries result = new TimeSeries(id)
        for(int i = 0;i<ts.getItemCount();i++) {
            def value = ts.getDataItem(i)
            result.add(value.setValue(function.call(value..getValue())))
        }
        result
    }
    
    static public TimeSeries apply(String id,TimeSeries ts1,TimeSeries ts2, Object function) {
        int min = Math.min(ts1.getItemCount(), ts2.getItemCount())
        TimeSeries result = new TimeSeries(id)
        for (int i=0; i<min;i++) {
            def val1 = ts1.getDataItem(i)
            def val2 = ts2.getDataItem(i)
            def val  = function.call(val1.getValue(),val2.getValue())
            result.add(val1.setValue(val))
        }
        result
    }
    
    static public double getTimeSeriesMax(String id) {
        return scripedPlot.getTimeSeriesFor(id).findValueRange().getUpperBound();
    }
    
    static public double getTimeSeriesMin(String id) {
        return scripedPlot.getTimeSeriesFor(id).findValueRange().getLowerBound();
    }
    
    static public void mark(double time, String label) {
        scripedPlot.mark(time,label)
    }

    static void title(String t) {
    	scripedPlot.title(t);
    }

}
