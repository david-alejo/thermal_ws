#include "glider_gui.h"

#include <QtGui/QLabel>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QAction>
#include <QtGui/QFileDialog>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QMessageBox>
#include <QtGui/QPainter>
#include <QImageWriter>
#include <QFile>

#ifdef _MSC_VER
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_symbol.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_renderer.h>
#else
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_marker.h>
#include <qwt/qwt_symbol.h>
#include <qwt/qwt_plot_canvas.h>
#include <qwt/qwt_plot_renderer.h>
#endif

#include "simulator/FlightPlan.h"
#include "functions/functions.h"
#include <UAVFlightPlan/UAVFlightPlan.h>

#include "About.h"
#include "ExecuteDialog.h"
// #include "MultiUAV.h"
#include "ScenarioDialog.h"
#include "AxisDialog.h"
#include "LatexDialog.h"
#include "CompleteSystemGui.h"

#include <sstream>
#include <iostream>
#include <QtGui/QInputDialog>

using namespace std;
using namespace functions;
using simulator::Updraft;
using glider_planner::URM;
using namespace Marble;

glider_gui::glider_gui()
{
    // Data initialize
    system = NULL;
    timer = NULL;
    layer = NULL;
    doc = NULL;
    logging = false;
  
    setupUi(this);
    connect(actionQuit, SIGNAL(triggered()), SLOT(close()));
    connect(actionLoad, SIGNAL(triggered()), SLOT(open()));
    connect(actionSave, SIGNAL(triggered()), SLOT(save()));
    connect(actionSave_As, SIGNAL(triggered()), SLOT(saveAs()));
    connect(actionExport_Plan, SIGNAL(triggered()), SLOT(exportPlan()));
    connect(actionExport_Constrains, SIGNAL(triggered(bool)), SLOT(exportConstrains()));
    connect(actionExecute, SIGNAL(triggered()), SLOT(execute()));
    connect(actionStart_Threads, SIGNAL(triggered()), SLOT(start_threads()));
    connect(actionAbout, SIGNAL(triggered()), SLOT(about()));
//     connect(actionScenario_Plot, SIGNAL(triggered()), SLOT(plotScenario()));
    connect(actionPlan_Plot, SIGNAL(triggered()), SLOT(plotPlan()));
    connect(actionSet_Axis, SIGNAL(triggered()), SLOT(setUserAxis()));
    connect(configTextEdit, SIGNAL(textChanged()), SLOT(enableSave()));
    connect(actionErase_Plot, SIGNAL(triggered()), SLOT(cleanPlot()));
    connect(actionSet_Marker_Size, SIGNAL(triggered()), SLOT(getMarkerSize()));
    
    // Set the map
    map_widget = new Marble::MarbleWidget(map_tab);
    map_widget->setProjection(Marble::Mercator);
    map_widget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    
    // Connections for changing the map style
    connect(actionBing, SIGNAL(triggered()), this, SLOT(setBingMap()));
    connect(actionGoogle_Maps, SIGNAL(triggered()), SLOT(setGoogleMapsMap()));
    connect(actionGoogle_Satellite, SIGNAL(triggered()), SLOT(setGoogleSatMap()));
    connect(actionOpen_Street_Map, SIGNAL(triggered()), SLOT(setOpenStreetMap()));
    
    
    // Create a horizontal zoom slider and set the default zoom
    QSlider * zoomSlider = new QSlider(Qt::Horizontal);
    zoomSlider->setMinimum( 1000 );
    zoomSlider->setMaximum( 2400 );
 
    map_widget->zoomView( zoomSlider->value() );
    
    map_widget->setWindowTitle("Mission and Wind map");
    map_widget->resize(1000, 700);
    map_widget->show();

    // Setting the default vertex symbol
    vertex_sym = new QwtSymbol;
    vertex_sym->setStyle(QwtSymbol::Diamond);
    vertex_sym->setPen(QPen(Qt::black));
    QBrush brush(Qt::red);
    vertex_sym->setBrush(brush);
    vertex_sym->setSize(marker_size);
    
    // Default edge pen
    QPen aux(Qt::black);
    edge_pen = aux;
    edge_pen.setWidth(3);
    
    QPen aux_2(Qt::blue);
    plan_pen = aux_2;
    plan_pen.setWidth(3);

    marker_size = 20;
    font_size = 16;
    
    disableSave();
    disableExecute();
}

bool glider_gui::open() {
  bool ret = true;
  QString fileName = QFileDialog::getOpenFileName(this, "Enter complete system filename");


  if (!fileName.isEmpty()) {
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
      QMessageBox::critical(this, tr("Error"),
      tr("Could not open file"));
      return false;
    }
    QString contents = file.readAll().constData();
    configTextEdit->setPlainText(contents);
    file.close();
    filename = fileName.toStdString();
    if (loadSystem()) {
      disableSave();
      statusbar->showMessage(tr("Complete system loaded successfully.\n"));
    } else {
      QMessageBox::critical(this, tr("Error"),
      tr("Scenario file not valid. Please correct the errors."));
      ret = false;
    }
  } else {
    ret = false;
  }

  return ret;
}

void glider_gui::saveAs()
{
  QString aux = QFileDialog::getSaveFileName(this);
  if (!aux.isEmpty()) {
    filename = aux.toStdString();
    enableSave();
    save();
  }
}

void glider_gui::save()
{
  QString aux(filename.c_str());
  QFile file(aux);
  if (!file.open(QIODevice::WriteOnly)) {
    QMessageBox::critical(this, tr("Error"),
    tr("Could not open file"));
    disableAll();
    return;
  }
    
  QTextStream stream(&file);
  stream << configTextEdit->toPlainText();
  stream.flush();
  file.close();
  if (!loadSystem()) {
    QMessageBox::critical(this, tr("Error"),
                          tr("Multi UAV scenario file not valid. Please correct the errors."));
  }
  disableSave();
}

// void glider_gui::exportGraph()
// {
  
// #ifndef QT_NO_PRINTER
//     QString fileName = "graph.svg";
/*#else
    QString fileName = "graph.png";
#endif

#ifndef QT_NO_FILEDIALOG
    const QList<QByteArray> imageFormats = QImageWriter::supportedImageFormats();

    QStringList filter;
    filter += "PDF Documents (*.pdf)";
#ifndef QWT_NO_SVG
    filter += "SVG Documents (*.svg)";
#endif
    filter += "Postscript Documents (*.ps)";

    if ( imageFormats.size() > 0 )
    {
        QString imageFilter("Images (");
        for ( int i = 0; i < imageFormats.size(); i++ )
        {
            if ( i > 0 )
                imageFilter += " ";
            imageFilter += "*.";
            imageFilter += imageFormats[i];
        }
        imageFilter += ")";

        filter += imageFilter;
    }

    fileName = QFileDialog::getSaveFileName(
        this, "Exporting Graph", fileName,
        filter.join(";;"), NULL, QFileDialog::DontConfirmOverwrite);
#endif
  if (fileName.isEmpty())
     return;

  QwtPlotRenderer renderer;
  
  renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
  renderer.setDiscardFlag(QwtPlotRenderer::DiscardCanvasBackground, false);
  renderer.renderDocument(graphPlot, fileName, QSizeF(300, 200), 85);
}*/

void glider_gui::exportPlan() {
  QString fileName = "plan.m";
   
  QStringList filter;
  filter += "Matlab Documents (*.m)";
  filter += "Latex Documents (*.tex)";
  filter += "Text Documents (*.txt)";
  filter += "QGroundControl (*.qgc)";
   
  fileName = QFileDialog::getSaveFileName(
        this, "Exporting Plan. Enter filename", fileName,
        filter.join(";;"), NULL);
   
  if (!fileName.isEmpty()) {
    QFile file(fileName);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    if (fileName.contains(".m")) {
      out << plan.toMatlab("plan").c_str();
    } else if (fileName.contains(".tex")) {
      std::string label, caption;
      LatexDialog::getTableData(this, label, caption);
      out << plan.toLatex(caption, label, 1).c_str();
    } else if (fileName.contains(".qgc")) {
      vector<simulator::FlightPlan> plans = system->getURM()->getFlightPlans();
      for (unsigned int i = 0; i < system->nUAVs(); i++) {
	QString filename_ = fileName.split(".").at(0);
	filename_.append(QString::number(i));
	filename_.append(".qgc");
	UAVFlightPlan::UAVFlightPlan fp(plans.at(i),system->getCenter(), true, plans.at(i).getReachAltitude());
	UAVFlightPlan::UAVFlightPlan::iterator fp_it = fp.begin();
	for (unsigned int j = 0; fp_it != fp.end(); fp_it++, j++) {
	  fp_it->setClimbRate(plans.at(i).getClimbRate(j));
	}
	fp.toQGCFile(filename_.toStdString());
      }
//       system->exportQGC(fileName.toStdString());
    } else {
      out << plan.toString().c_str();
    }
  }
}

void glider_gui::exportConstrains() {
  QString fileName = "constrains.tex";
   
  QStringList filter;
  filter += "Latex Documents (*.tex)";
  filter += "Matlab Documents (*.m)";
  filter += "Text Documents (*.txt)";
   
  fileName = QFileDialog::getSaveFileName(
        this, "Exporting Constraints. Enter filename", fileName,
        filter.join(";;"), NULL);
   
  if (system != NULL && !fileName.isEmpty()) {
    const URM *urm = system->getURM();
    QFile file(fileName);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    
    if (fileName.contains(".m")) {
      out << urm->toString().c_str();
//       out << urm->toMatlab("constrains").c_str();
    } else if (fileName.contains(".tex")) {
      std::string label, caption;
      LatexDialog::getTableData(this, label, caption);
      out << urm->toLatex(caption, label, 1).c_str();
    } else {
      out << urm->toString().c_str();
    }
      
    file.close();
  }
}

void glider_gui::setPlotStyle(QwtPlot *graph) {
//   QStyle graph->style()
}

bool glider_gui::loadSystem() {
  if (layer != NULL) {
    map_widget->removeLayer(layer);
    delete layer;
    layer = NULL;
  }
  
  delete system;
  system = new CompleteSystemGui(filename, this);
  
  actionExecute->setEnabled(system->isLoaded());
  
  paintScenario();
  
  return system->isLoaded(); 
}

void glider_gui::disableAll()
{
  enable_memory.clear();
  enable_memory.push_back(actionLoad->isEnabled());
  enable_memory.push_back(actionSave->isEnabled());
  enable_memory.push_back(actionSave_As->isEnabled());
  enable_memory.push_back(actionExecute->isEnabled());
  enable_memory.push_back(actionExecute_2->isEnabled());
  
  disableSave();
  disableLoad();
  disableExecute();
}

void glider_gui::disableLoad()
{
  actionLoad->setEnabled(false);
  actionSave_As->setEnabled(false);
}


void glider_gui::disableSave()
{
  actionSave->setEnabled(false);
}

void glider_gui::disableExecute()
{
  actionExecute->setEnabled(false);
  actionExecute_2->setEnabled(false);
//   actionSave_As->setEnabled(false);
}

void glider_gui::enableExecute()
{
  actionExecute->setEnabled(true);
  actionExecute_2->setEnabled(true);
}


void glider_gui::enableSave()
{
  actionSave->setEnabled(true);
//   actionSave_As->setEnabled(true);
}

void glider_gui::enableAll()
{
  if (enable_memory.empty()) {
    enableSave();
    enableExecute();
    enableLoad();
  } else {
    actionLoad->setEnabled(enable_memory.at(0));
    actionSave->setEnabled(enable_memory.at(1));
    actionSave_As->setEnabled(enable_memory.at(2));
    actionExecute->setEnabled(enable_memory.at(3));
    actionExecute_2->setEnabled(enable_memory.at(4));
  }
}

void glider_gui::enableLoad()
{
  actionLoad->setEnabled(true);
  actionSave_As->setEnabled(true);
}

void glider_gui::start_threads()
{
  system->start_threads();
  initializeLocations();
  initializeUAVWidgets();
}

void glider_gui::initializeLocations()
{
  if (!logging) {
    logging = true;
  } else {
    return;
  }
  system->initializeLocationTable();
  system->initializeWaypointTable();
  initializePlacemarcks();
  
  // Gathering position data
  connect(system, SIGNAL(UAVCoordinatesChanged(Marble::GeoDataCoordinates,uint)), 
	  this, SLOT(updateUAVPlacemark(Marble::GeoDataCoordinates,uint)));
  
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), system, SLOT(update()));
  timer->setInterval(int(system->getSleepTime()*1000));
  timer->start();
}

void glider_gui::initializeUAVWidgets()
{
   connect(system, SIGNAL(UAVStateChanged(glider_planner::UAVState, uint)),
           this, SLOT(updateUAVState(glider_planner::UAVState,uint)));   
   
  for(int i=0;i<system->nUAVs();i++) {
    
    UAS_Widget * new_uas_wid = new UAS_Widget(this, system->getUAV(i));
    verticalLayout_5->addWidget(new_uas_wid);
    uas_widget_vector.push_back(new_uas_wid);
   
    
  }
    
}  

void glider_gui::updateUAVState(glider_planner::UAVState st, uint n_uav)
{
  uas_widget_vector[n_uav]->updateUAVState(st);
}


void glider_gui::execute() {
  if (system == NULL) {
      return;
    }
    
  disableExecute();
  disableSave();
  disableLoad();
  
  system->execute();
  
  initializeLocations();
}

glider_gui::~glider_gui()
{
  
  if (timer != NULL) {
    system->stop();
    delete timer;
  }
  
  delete vertex_sym;
  vertex_sym = NULL;
  delete system;
  system = NULL;
  delete map_widget;
  map_widget = NULL;
}

void glider_gui::about()
{
      QDialog *diag = new About(this);
      diag->show();
}

// void glider_gui::paintGraph()
// {
//   // Erase graph
//   graphPlot->detachItems(QwtPlotItem::Rtti_PlotCurve, true);
//   graphPlot->detachItems(QwtPlotItem::Rtti_PlotMarker, true);
//   
//   
//   if (planner != NULL) {
//     const Graph &g = planner->getGraph();
//     
//     // Print graph details in Detail tab
//     ostringstream os;
//     os << "Graph details: " << endl << endl;
//     os << g.toString() << endl;
//     os << endl << "Locations of the graph:" << endl;
//   
//   
//     for (int i = 0;i < g.nVertices(); i++) {
// //     std::cout << i << std::endl;
//       if (g.nEdges(i) == 0) {
// 	continue; // Do not represent unconnected vertices
//       }
// 
//       functions::RealVector v(g.getVertexContent(i));
//       QwtPlotMarker *mark = new QwtPlotMarker();
//       QPointF init_node(v.at(0), v.at(1));
//       mark->setValue(init_node);
//       mark->setSymbol(new QwtSymbol(*vertex_sym));
//       mark->attach(graphPlot);
//     
//       os << "Node " << i << ": " << v.toString() << endl;
// //     mark->
//     
//       std::list<int> edges = g.getEdges(i);
//       list<int>::iterator it = edges.begin();
//       for (;it != edges.end(); it++) {
// 	// Then the edges
// 	// add curve
// 	QVector<QPointF> vec;
// 	QwtPlotCurve *curve1 = new QwtPlotCurve("Graph");
// 	vec.push_back(init_node);
// 	functions::RealVector v(g.getVertexContent(*it));
// 	QPointF goal_node(v.at(0), v.at(1));
// 	vec.push_back(goal_node);
// 	
// 	curve1->setSamples(vec);
// 	// Curve style
// 	curve1->setPen(edge_pen);
// 	curve1->attach(graphPlot);
//       }
//     }
//     graphPlot->replot();
//   
//     textBrowser->setText(os.str().c_str());
//   }
// }

void glider_gui::plotPlan() {
  if (system != NULL) {
    const URM *urm = system->getURM();
    const std::vector<simulator::FlightPlan> &plans = urm->getPlans();
    for (unsigned int i = 0; i < plans.size(); i++) {
      QPen pen = plan_pen;
      switch(i) {
        case 0:
          pen.setColor(Qt::red);
          break;
        case 1:
          pen.setColor(Qt::blue);
          break;
        case 2:
          pen.setColor(Qt::black);
          break;
        case 3:
          pen.setColor(Qt::green);
          break;
        default:
          pen.setColor(Qt::gray);
      }
      paintPlan(pen, plans.at(i));
      RealVector lower_world = system->getMinimumBounds();
      RealVector upper_world = system->getMaximumBounds();
      graphPlot->setAxisScale(2, lower_world.at(0), upper_world.at(0));
      graphPlot->setAxisScale(0, lower_world.at(1), upper_world.at(1));
      graphPlot->replot();
    }
  }
}

void glider_gui::paintPlan(const QPen &plan_pen, const simulator::FlightPlan &plan)
{
  
  simulator::FlightPlan::const_iterator it = plan.begin();
  QVector<QPointF> vec;
  for (;it < plan.end(); it++) {
//     std::cout << i << std::endl;
    QPointF node(it->x, it->y);
    vec.push_back(node);
  }
  if (vec.size() > 0) {
    QwtPlotCurve *curve1 = new QwtPlotCurve("Graph");
    curve1->setSamples(vec);
    // Curve style
    curve1->setPen(plan_pen);
    curve1->attach(graphPlot);
  
  
    // Mark the initial nodes
    QwtSymbol *init_symbol = new QwtSymbol;
    init_symbol->setStyle(QwtSymbol::Diamond);
    init_symbol->setPen(QPen(Qt::black));
    QBrush brush(Qt::green);
    init_symbol->setBrush(brush);
    init_symbol->setSize(marker_size);
  
    QwtPlotMarker *goal_mark = new QwtPlotMarker();
    goal_mark->setValue(vec.at(0));
    goal_mark->setSymbol(init_symbol);
    goal_mark->attach(graphPlot);
  
  
    // And then the goal node
  
    QBrush brush_2(Qt::blue);
    init_symbol->setBrush(brush_2);
    QwtPlotMarker *mark = new QwtPlotMarker();
    mark->setValue(vec.at(vec.size() - 1));
    mark->setSymbol(init_symbol);
    mark->attach(graphPlot);
  }
  
  // Replot (last action)
  graphPlot->replot();
//  RealVector lower_world = planner->getLowerWorld();
//  RealVector upper_world = planner->getUpperWorld();
//   graphPlot->setAxisScale(0, lower_world.at(0), upper_world.at(0));
//   graphPlot->setAxisScale(1, lower_world.at(1), upper_world.at(1));
}

void glider_gui::paintWaypoints(const vector<RealVector> &ways) {
//   std::vector<RealVector>::const_iterator it = ways.begin();
//   int cont = 1;
  
//   for (;it != ways.end(); it++, cont++) {
//     UAVFlightPlan::EarthLocation loc = getLocation(it->at(0), it->at(1));
//     map_widget->adaddMarker(Marker::WaypointMarker, loc, waypoint_marker_icon);
//     graphPlot->
//   }
  
  graphPlot->replot();
}


void glider_gui::processExecution()
{
  // Repaintsthe graph 
  std::cout << plan.toString() << std::endl;
  plotPlan(); // marks the visited edge with another color
  actionExport_Plan->setEnabled(true); // Enable export plan
  ostringstream os;
  os << plan.toString() << endl;
  
  os << "Path turning angles: \n" << plan.printAngles() << endl;
    
  textBrowser->setText(tr(os.str().c_str()));
}

void glider_gui::processSystemExecution() {
  // Repaintsthe graph
  actionExport_Plan->setEnabled(true); // Enable export plan

  // Set the text of the details tab
  ostringstream os;
  os << system->getURM()->toString();
  textBrowser->setText(tr(os.str().c_str()));
}

void glider_gui::paintScenario()
{
  bool paint_way, paint_updraft, clean_plot;
  
  if (system != NULL && ScenarioDialog::getScenarioDialogResults(this, paint_way,  paint_updraft, clean_plot, updraft_marker_icon, waypoint_marker_icon, uav_icon)) {
    if (clean_plot) {
      cleanPlot();
    }
    
    // Create and register our paint layer
    
    if (layer != NULL) {
      map_widget->removeLayer(layer);
    }
    
    layer = new ScenarioLayer(map_widget, system, uav_icon, paint_updraft, paint_way);
    // Uncomment for older versions of Marble:
    map_widget->addLayer(layer);

  } 
}

void glider_gui::cleanPlot()
{
  graphPlot->detachItems(QwtPlotItem::Rtti_PlotCurve, true);
  graphPlot->detachItems(QwtPlotItem::Rtti_PlotMarker, true);
}

void glider_gui::setUserAxis()
{
  double min_x = graphPlot->axisInterval(2).minValue();
  double min_y = graphPlot->axisInterval(0).minValue();
  double max_x = graphPlot->axisInterval(2).maxValue();
  double max_y = graphPlot->axisInterval(0).maxValue();
  
  if (AxisDialog::getNewAxisLimits(this, min_x, min_y, max_x, max_y)) {
    graphPlot->setAxisScale(2, min_x, max_x);
    graphPlot->setAxisScale(0, min_y, max_y);
    graphPlot->replot();
  }
}

void glider_gui::getMarkerSize() {
  bool ok;
  int aux = QInputDialog::getInteger(this, tr("Enter new marker size"), tr("Marker Size"), marker_size,
                                     8, 30, 1, &ok);

  if (ok) {
    marker_size = aux;
  }
}

UAVFlightPlan::EarthLocation glider_gui::getLocation(double north_shift, double east_shift)
 {
      UAVFlightPlan::EarthLocation e;
      if (system != NULL) {
	e = system->getCenter();
	
	e.shift(north_shift, east_shift);
      }
      return e;
    }

    
void glider_gui::initializePlacemarcks()
{
  if (system == NULL) {
    return;
  }
  vector<UAVFlightPlan::EarthLocation> locs = system->getPositions();
  
  if (doc != NULL) {
    map_widget->model()->treeModel()->removeDocument(doc);
    doc = NULL;
    uav_placemarcks.clear();
  }
  
  doc = new GeoDataDocument;
  
  for (unsigned int i = 0; i < locs.size(); i++) {
    QString uav_name("UAV");
    uav_name.append(QString::number(i));
    GeoDataCoordinates coord = ScenarioLayer::toMarble(locs.at(i));
    GeoDataPlacemark * uav_placemarck = new GeoDataPlacemark(uav_name);
    uav_placemarck->setCoordinate(coord);
    doc->append(uav_placemarck);
    uav_placemarcks.push_back(uav_placemarck);
  }
  
  map_widget->model()->treeModel()->addDocument(doc);
}

void glider_gui::updateUAVPlacemark(GeoDataCoordinates coord, unsigned int i)
{
  if (system == NULL || (int)i >= system->nUAVs()) {
    return;
  }
  
  uav_placemarcks.at(i)->setCoordinate(ScenarioLayer::toMarble( system->getPosition(i)));
  
}

void glider_gui::setBingMap()
{
  map_widget->setMapThemeId("earth/virtualearth/virtualearth.dgml");
}

void glider_gui::setGoogleMapsMap()
{
  map_widget->setMapThemeId("earth/googlemaps/googlemaps.dgml");
}

void glider_gui::setGoogleSatMap()
{
  map_widget->setMapThemeId("earth/googlesat/googlesat.dgml");
}

void glider_gui::setOpenStreetMap()
{
  map_widget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
}
