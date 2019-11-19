from utility import *


class MouseInteractorHighLightActor(vtk.vtkInteractorStyleTrackballCamera):

    def __init__(self, parent=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.AddObserver("LeftButtonPressEvent", self.leftButtonPressEvent)

        self.select_queue = []
        self.LastPickedActor = None
        self.NewPickedActor = None
        self.LastPickedProperty = vtk.vtkProperty()

    def leftButtonPressEvent(self, obj, event):
        clickPos = self.GetInteractor().GetEventPosition()

        picker = vtk.vtkPropPicker()
        picker.Pick(clickPos[0], clickPos[1], 0, self.GetDefaultRenderer())

        # get the new
        self.NewPickedActor = picker.GetActor()

        # If something was selected
        if self.NewPickedActor:
            # If we picked something before, reset its property
            if self.LastPickedActor:
                self.LastPickedActor.GetProperty().DeepCopy(self.LastPickedProperty)

            # Save the property of the picked actor so that we can
            # restore it next time
            self.LastPickedProperty.DeepCopy(self.NewPickedActor.GetProperty())
            # Highlight the picked actor by changing its properties
            self.NewPickedActor.GetProperty().SetColor(1.0, 0.0, 0.0)
            self.NewPickedActor.GetProperty().SetDiffuse(1.0)
            self.NewPickedActor.GetProperty().SetSpecular(0.0)

            # save the last picked actor
            self.LastPickedActor = self.NewPickedActor

            # add to queue
            if len(self.select_queue) is 2:
                self.select_queue.pop(0)
            self.select_queue.append(self.NewPickedActor)

        self.OnLeftButtonDown()
        return


class vtkTimerEvent:

    def __init__(self, renderer: vtk.vtkRenderer, listener: MouseInteractorHighLightActor, mesh: Mesh):
        self.timerId = None
        self.renderer = renderer
        self.listener = listener
        self.mesh = mesh

    def execute(self, iren, event):
        # iren.DestroyTimer(self.timerId)
        # self.renderer.ResetCamera()

        if len(listener.select_queue) is 2:
            first = listener.select_queue.pop(0)
            second = listener.select_queue.pop(0)

            left = second
            right = first
            if (first.GetBounds()[0] - second.GetBounds()[0]) > 0:
                left = first
                right = second

            x0, x1, y0, y1, z0, z1 = left.GetBounds()
            f = ((axes[0] + (x0 + x1) / 2) / 2, (axes[1] + (y0 + y1) / 2) / 2, 0)

            x0, x1, y0, y1, z0, z1 = right.GetBounds()
            s = ((axes[0] + (x0 + x1) / 2) / 2, (axes[1] + (y0 + y1) / 2) / 2, 0)

            self.renderer.AddActor(self.mesh.draw(f, s, w0=axes0))

        iren.GetRenderWindow().Render()


if __name__ == "__main__":
    colors = vtk.vtkNamedColors()
    backgroundColor = colors.GetColor3d("Black")

    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()

    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(1080, 720)

    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    """
    Apply DPA on Mesh to get optimal path (average of per 10px etc)
    https://www.mathworks.com/matlabcentral/fileexchange/39034-finding-optimal-path-on-a-terrain
    """

    # Start Scene

    # Elements
    axes = (54, 29, 30)
    axes0 = tuple(-x for x in axes)

    axes_3d = Axes(renderer, colors, axes=axes)
    _mesh = Mesh(renderer, colors, axes[0], axes[1], file='data/DTM.csv', distance_diff=0.5, scale=1)
    _waypoints = Points(renderer, colors, file='data/Waypoints.csv').draw(w0=axes0)
    _landmarks = Points(renderer, colors, file='data/Landmarks.csv').draw(w0=axes0)
    mesh = _mesh.get_mesh(w0=axes0)
    line = _mesh.draw((17.70, 1.50, -0.20), (3.60, 12.70, -0.30), w0=axes0)

    # Create vectors
    arm = Vector(renderer, colors, endPoint=[10, 0, 0], color="Red")
    camera = Vector(renderer, colors, startPoint=[5, 0, 5], endPoint=[10, 0, 4], color="Cyan")
    lidar = Vector(renderer, colors, startPoint=[0, 5, -3], endPoint=[5, 5, -3], color="Tomato")

    # Text
    text = Text(renderer, colors).draw("Autonomous", 1, 1, 1)

    # Add actors
    renderer.AddActor(arm.get_actor())
    renderer.AddActor(camera.get_actor())
    renderer.AddActor(lidar.get_actor())
    renderer.AddActor(axes_3d.get_actor())
    renderer.AddActor(line)
    renderer.AddActor(mesh)
    renderer.AddActor(text)

    renderer.GetActiveCamera().Pitch(90)
    renderer.GetActiveCamera().Azimuth(-90)
    renderer.ResetCamera()
    renderer.SetBackground(backgroundColor)
    renderWindow.Render()

    # add the mouse click listener
    listener = MouseInteractorHighLightActor()
    listener.SetDefaultRenderer(renderer)
    renderWindowInteractor.SetInteractorStyle(listener)

    # Initialize a timer for the animation
    timer = vtkTimerEvent(renderer, listener, _mesh)
    renderWindowInteractor.AddObserver('TimerEvent', timer.execute)
    timerId = renderWindowInteractor.CreateRepeatingTimer(300)
    timer.timerId = timerId

    renderWindowInteractor.Start()
