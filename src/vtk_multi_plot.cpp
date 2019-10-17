

#include <vtkChartXY.h>
#include <vtkChartMatrix.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkFloatArray.h>
#include <vtkNamedColors.h>
#include <vtkPlotPoints.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTable.h>


static vtkSmartPointer<vtkChartXY> create_ChartXY(double offset)
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

  auto chartXY = vtkSmartPointer<vtkChartXY>::New();

  // Add multiple scatter plots, setting the colors etc
  auto points = chartXY->AddPlot(vtkChart::POINTS);
  points->SetInputData(table, 0, 1);
  points->SetColor(255, 0, 0, 255);
  points->SetWidth(1.0);
  dynamic_cast<vtkPlotPoints *>(points)->SetMarkerStyle(vtkPlotPoints::CROSS);

  points = chartXY->AddPlot(vtkChart::POINTS);
  points->SetInputData(table, 0, 2);
  points->SetColor(0, 0, 255, 255);
  points->SetWidth(1.0);
  dynamic_cast<vtkPlotPoints *>(points)->SetMarkerStyle(vtkPlotPoints::PLUS);

  return chartXY;
}

int vtk_multi_plot()
{
  int num_rows = 2;
  int num_cols = 3;

  auto colors = vtkSmartPointer<vtkNamedColors>::New();

  auto matrix = vtkSmartPointer<vtkChartMatrix>::New();;
  matrix->SetSize(vtkVector2i(num_cols, num_rows));
  matrix->SetGutter(vtkVector2f(45.0, 45.0));

  for (int r = 0; r < num_rows; r += 1)
    for (int c = 0; c < num_cols; c += 1) {

      auto xyPlot = create_ChartXY(c);
      matrix->SetChart(vtkVector2i(c, r), xyPlot);
    }

  auto view = vtkSmartPointer<vtkContextView>::New();
  view->GetRenderWindow()->SetSize(1280, 1024);
  view->GetScene()->AddItem(matrix);

  // Start interactor
  view->GetRenderWindow()->Render();
  view->GetInteractor()->Initialize();
  view->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
