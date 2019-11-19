import vtk
from numpy import random


class VtkPointCloud:

    def __init__(self, zMin=-10.0, zMax=10.0, maxNumPoints=1e6):
        self.maxNumPoints = maxNumPoints
        self.vtkPolyData = vtk.vtkPolyData()
        self.clearPoints()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.vtkPolyData)
        mapper.SetColorModeToDefault()
        mapper.SetScalarRange(zMin, zMax)
        mapper.SetScalarVisibility(1)
        self.vtkActor = vtk.vtkActor()
        self.vtkActor.SetMapper(mapper)

    def addPoint(self, point):
        if self.vtkPoints.GetNumberOfPoints() < self.maxNumPoints:
            pointId = self.vtkPoints.InsertNextPoint(point[:])
            self.vtkDepth.InsertNextValue(point[2])
            self.vtkCells.InsertNextCell(1)
            self.vtkCells.InsertCellPoint(pointId)
        else:
            r = random.randint(0, self.maxNumPoints)
            self.vtkPoints.SetPoint(r, point[:])
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()

    def clearPoints(self):
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()
        self.vtkDepth.SetName('DepthArray')
        self.vtkPolyData.SetPoints(self.vtkPoints)
        self.vtkPolyData.SetVerts(self.vtkCells)
        self.vtkPolyData.GetPointData().SetScalars(self.vtkDepth)
        self.vtkPolyData.GetPointData().SetActiveScalars('DepthArray')


class AddPointCloudTimerCallback:

    def __init__(self, renderer, iterations):
        self.iterations = iterations
        self.renderer = renderer

    def execute(self, iren, event):
        if self.iterations == 0:
            iren.DestroyTimer(self.timerId)
        pointCloud = VtkPointCloud()
        self.renderer.AddActor(pointCloud.vtkActor)
        pointCloud.clearPoints()
        for k in range(10000):
            point = 20 * (random.rand(3) - 0.5)
            pointCloud.addPoint(point)
        pointCloud.addPoint([0, 0, 0])
        pointCloud.addPoint([0, 0, 0])
        pointCloud.addPoint([0, 0, 0])
        pointCloud.addPoint([0, 0, 0])
        iren.GetRenderWindow().Render()
        if self.iterations == 30:
            self.renderer.ResetCamera()

        self.iterations -= 1


if __name__ == "__main__":
    # Renderer
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(.2, .3, .4)
    renderer.ResetCamera()

    # Render Window
    renderWindow = vtk.vtkRenderWindow()

    renderWindow.AddRenderer(renderer)

    # Interactor
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)
    renderWindowInteractor.Initialize()

    # Initialize a timer for the animation
    addPointCloudTimerCallback = AddPointCloudTimerCallback(renderer, 300)
    renderWindowInteractor.AddObserver('TimerEvent', addPointCloudTimerCallback.execute)
    timerId = renderWindowInteractor.CreateRepeatingTimer(50)
    addPointCloudTimerCallback.timerId = timerId

    # Begin Interaction
    renderWindow.Render()
    renderWindowInteractor.Start()
