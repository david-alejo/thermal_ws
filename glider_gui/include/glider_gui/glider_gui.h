#ifndef glider_gui_H
#define glider_gui_H

#include <glider_planner/SoaringPlanner.h>
#include <glider_planner/URM.h>
#include <glider_planner/CompleteSystem.h>
#include "glider_planner/UAVState.h"

#include <UAVFlightPlan/earthlocation.h>

#include <QtGui/QMainWindow>
#include <QtGui/QSpinBox>
#include <QTimer>
#ifndef _MSC_VER
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_marker.h>
#include <qwt/qwt_symbol.h>
#include "ui_glider_planner.h"
#include "UAS_Widget.h"
#else
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_symbol.h>
#include "../vs/ui_glider_planner.h"
#endif

#include "CompleteSystemGui.h"

// Maps widget
#include <marble/MarbleWidget.h>
#include "ScenarioLayer.h"

#include <marble/GeoDataPlacemark.h>
#include <marble/GeoDataLineString.h>
#include <marble/GeoDataTreeModel.h>
#include <marble/GeoDataDocument.h>
#include <marble/MarbleModel.h>

#include <boost/thread.hpp>

class CompleteSystemGui;
class ScenarioLayer;

class glider_gui : public QMainWindow, Ui::GliderGUI
{
Q_OBJECT
public:
    glider_gui(); 
    virtual ~glider_gui();
    
    void paintUpdrafts(const std::vector< simulator::Updraft >& ups);
    void paintWaypoints(const std::vector< functions::RealVector >& ways);
    void paintPlan(const QPen &pen, const simulator::FlightPlan &plan);
    
    inline void updateMap() {
      if (map_widget != NULL) {
	map_widget->repaint();
      }
    }
    
public slots:
  void updateUAVPlacemark(Marble::GeoDataCoordinates coord, unsigned int i);
  void updateUAVState(glider_planner::UAVState st, unsigned int n_uav);
 private slots:
//      void newFile();
     bool open();
     void execute();
     void start_threads();
     void save();
     void saveAs();
     void about();
     void cleanPlot();
     void enableSave();
     void exportPlan();
     void exportConstrains();
     void paintScenario();
     void plotPlan();
     void setUserAxis();
     void getMarkerSize();
     
    
private:
    CompleteSystemGui *system;
    simulator::FlightPlan plan;
    std::string filename;
    ScenarioLayer*layer;
    
    // Timer for data update 
    QTimer *timer;
    
    // Qwt Styles
    QwtSymbol *vertex_sym;
    QPen edge_pen;
    QPen plan_pen;
    int marker_size;
    int font_size;
    
    // Map related
    QString waypoint_marker_icon, updraft_marker_icon, uav_icon;
    Marble::MarbleWidget *map_widget;
    std::vector<Marble::GeoDataPlacemark *> uav_placemarcks;
    Marble::GeoDataDocument *doc;
    
    // UAV State
    std::vector <UAS_Widget*>  uas_widget_vector;
    
    //! @brief Loads a system from filename field
    bool loadSystem();
    
    //! @brief Sets the graph, plan tabs and the execute action to disable
    void disableAll();
    void disableSave();
    void disableLoad();
//     void disableGraph();
    void disableExecute();
    void enableExecute();
    void enableLoad();
    void enableAll();
    std::vector<bool> enable_memory;
    bool logging;
       
    void setPlotStyle(QwtPlot *graph);
    
    void processExecution();

    void processSystemExecution();
    
    UAVFlightPlan::EarthLocation getLocation(double north_shift, double east_shift);
    
    void initializePlacemarcks();
    
    void clearPlacemarcks();
    
    friend class CompleteSystemGui;
protected:
    void initializeLocations();
    void initializeUAVWidgets();
};

#endif // glider_gui_H
