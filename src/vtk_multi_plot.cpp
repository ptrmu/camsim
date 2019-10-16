

#include <vtkFloatArray.h>
#include <vtkNamedColors.h>
#include <vtkPlotPoints.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTable.h>
#include <vtkXYPlotActor.h>


static vtkSmartPointer<vtkXYPlotActor> create_XYPlot()
{
  // Create a table with some points in it
  auto table = vtkSmartPointer<vtkTable>::New();

  auto arrX = vtkSmartPointer<vtkFloatArray>::New();
  arrX->SetName("X Axis");
  table->AddColumn(arrX);

  auto arrC = vtkSmartPointer<vtkFloatArray>::New();
  arrC->SetName("Cosine");
  table->AddColumn(arrC);

  auto arrS = vtkSmartPointer<vtkFloatArray>::New();
  arrS->SetName("Sine");
  table->AddColumn(arrS);

  // Fill in the table with some example values
  int numPoints = 69;
  float inc = 7.5 / (numPoints - 1);
  table->SetNumberOfRows(numPoints);
  for (int i = 0; i < numPoints; ++i) {
    table->SetValue(i, 0, i * inc);
    table->SetValue(i, 1, cos(i * inc));
    table->SetValue(i, 2, sin(i * inc));
  }

  auto xyPlot = vtkSmartPointer<vtkXYPlotActor>::New();

  // Add multiple scatter plots, setting the colors etc
//  auto points = xyPlot->AddPlot(vtkChart::POINTS);
//  points->SetInputData(table, 0, 1);
//  points->SetColor(0, 0, 0, 255);
//  points->SetWidth(1.0);
//  dynamic_cast<vtkPlotPoints*>(points)->SetMarkerStyle(vtkPlotPoints::CROSS);
//
//  points = xyPlot->AddPlot(vtkChart::POINTS);
//  points->SetInputData(table, 0, 2);
//  points->SetColor(0, 0, 0, 255);
//  points->SetWidth(1.0);
//  dynamic_cast<vtkPlotPoints*>(points)->SetMarkerStyle(vtkPlotPoints::PLUS);

  xyPlot->
  return xyPlot;
}

int vtk_multi_plot()
{
  vtkSmartPointer<vtkNamedColors> colors =
    vtkSmartPointer<vtkNamedColors>::New();

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();

  for (int r = 0; r < 6; r += 1)
    for (int c = 0; c < 6; c += 1) {

      auto xyPlot = create_XYPlot();

      auto renderer = vtkSmartPointer<vtkRenderer>::New();
      renderer->AddActor(xyPlot);

    }

}
