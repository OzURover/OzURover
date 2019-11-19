import vtk
import numpy as np
import math
import colorsys


class Utility:

    def __init__(self):
        self.start = None


class Object:

    def __init__(self, renderer, colors):
        self.renderer = renderer
        self.colors = colors


class Axes(Object):

    def __init__(self, renderer, colors, axes=(0, 0, 0), xe=None, ye=None, ze=None, single=None):
        super().__init__(renderer, colors)
        self.actor = vtk.vtkCubeAxesActor()
        self.x = axes[0]
        self.y = axes[1]
        self.z = axes[2]
        if single is not None:
            self.x = self.y = self.z = single
        if xe is None:
            self.xe = self.x * -1
        if ye is None:
            self.ye = self.y * -1
        if ze is None:
            self.ze = self.z * -1
        self.draw()

    def draw(self):
        axis1Color = self.colors.GetColor3d("Salmon")
        axis2Color = self.colors.GetColor3d("PaleGreen")
        axis3Color = self.colors.GetColor3d("DodgerBlue")

        self.actor.SetUseTextActor3D(1)
        self.actor.SetBounds(self.xe, self.x, self.ye, self.y, self.ze, self.z)
        self.actor.SetCamera(self.renderer.GetActiveCamera())
        self.actor.GetTitleTextProperty(0).SetColor(axis1Color)
        self.actor.GetTitleTextProperty(0).SetFontSize(48)
        self.actor.GetLabelTextProperty(0).SetColor(axis1Color)

        self.actor.GetTitleTextProperty(1).SetColor(axis2Color)
        self.actor.GetLabelTextProperty(1).SetColor(axis2Color)

        self.actor.GetTitleTextProperty(2).SetColor(axis3Color)
        self.actor.GetLabelTextProperty(2).SetColor(axis3Color)

        self.actor.DrawXGridlinesOn()
        self.actor.DrawYGridlinesOn()
        self.actor.DrawZGridlinesOn()
        self.actor.SetGridLineLocation(self.actor.VTK_GRID_LINES_FURTHEST)

        self.actor.XAxisMinorTickVisibilityOff()
        self.actor.YAxisMinorTickVisibilityOff()
        self.actor.ZAxisMinorTickVisibilityOff()

        self.actor.SetFlyModeToStaticEdges()

    def get_actor(self):
        return self.actor

    def update(self, x=0, y=0, z=0, xe=None, ye=None, ze=None, single=None):
        self.x = x
        self.y = y
        self.z = z
        if single is not None:
            self.x = self.y = self.z = single
        if xe is None:
            self.xe = self.x * -1
        if ye is None:
            self.ye = self.y * -1
        if ze is None:
            self.ze = self.z * -1

        self.renderer.RemoveActor(self.actor)

        self.actor = vtk.vtkCubeAxesActor()
        self.draw()

        self.renderer.AddActor(self.actor)


class Vector(Object):

    def __init__(self, renderer, colors, startPoint=None, endPoint=None, color="Cyan"):
        super().__init__(renderer, colors)
        self.actor = vtk.vtkActor()
        self.startPoint = startPoint
        self.endPoint = endPoint
        self.color = color
        self.draw()

    def draw(self):
        if self.startPoint is None:
            self.startPoint = [0, 0, 0]

        # Create an arrow.
        arrowSource = vtk.vtkArrowSource()

        rng = vtk.vtkMinimalStandardRandomSequence()

        # Compute a basis
        normalizedX = [0] * 3
        normalizedY = [0] * 3
        normalizedZ = [0] * 3

        # The X axis is a vector from start to end
        vtk.vtkMath.Subtract(self.endPoint, self.startPoint, normalizedX)
        length = vtk.vtkMath.Norm(normalizedX)
        vtk.vtkMath.Normalize(normalizedX)

        # The Z axis is an arbitrary vector cross X
        arbitrary = [0] * 3
        for i in range(0, 3):
            rng.Next()
            arbitrary[i] = rng.GetRangeValue(-10, 10)
        vtk.vtkMath.Cross(normalizedX, arbitrary, normalizedZ)
        vtk.vtkMath.Normalize(normalizedZ)

        # The Y axis is Z cross X
        vtk.vtkMath.Cross(normalizedZ, normalizedX, normalizedY)
        matrix = vtk.vtkMatrix4x4()

        # Create the direction cosine matrix
        matrix.Identity()
        for i in range(0, 3):
            matrix.SetElement(i, 0, normalizedX[i])
            matrix.SetElement(i, 1, normalizedY[i])
            matrix.SetElement(i, 2, normalizedZ[i])

        # Apply the transforms
        transform = vtk.vtkTransform()
        transform.Translate(self.startPoint)
        transform.Concatenate(matrix)
        transform.Scale(length, length, length)

        # Transform the polydata
        transformPD = vtk.vtkTransformPolyDataFilter()
        transformPD.SetTransform(transform)
        transformPD.SetInputConnection(arrowSource.GetOutputPort())

        # Create a mapper and actor for the arrow
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(arrowSource.GetOutputPort())
        self.actor.SetUserMatrix(transform.GetMatrix())
        self.actor.SetMapper(mapper)
        self.actor.GetProperty().SetColor(self.colors.GetColor3d(self.color))

    def get_actor(self):
        return self.actor

    def update(self, startPoint=None, endPoint=None):
        self.startPoint = startPoint
        self.endPoint = endPoint

        self.renderer.RemoveActor(self.actor)

        self.actor = vtk.vtkActor()
        self.draw()

        self.renderer.AddActor(self.actor)


class Mesh(Object):

    def __init__(self, renderer, colors, x, y, file, distance_diff=1, scale=2):
        super().__init__(renderer, colors)
        self.prev_actor = None
        self.distance_diff = distance_diff
        x_size = int(x / distance_diff) + 1
        y_size = int(y / distance_diff) + 1
        self.z_max = 0
        self.z_min = 0

        topography = np.zeros((x_size, y_size))

        with open(file) as fp:
            fp.readline()
            line = fp.readline()
            while line:
                row = [float(x) for x in line.strip().split(",")]
                x = int(row[0] / distance_diff)
                y = int(row[1] / distance_diff)
                z = row[2] / distance_diff * scale

                if z > self.z_max:
                    self.z_max = z

                if z < self.z_min:
                    self.z_min = z

                topography[x][y] = z
                line = fp.readline()

        # Define points, triangles and colors
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        points = vtk.vtkPoints()
        triangles = vtk.vtkCellArray()

        # Build the meshgrid manually.
        count = 0
        for i in range(x_size - 1):
            for j in range(y_size - 1):
                z1 = topography[i][j]
                z2 = topography[i][j + 1]
                z3 = topography[i + 1][j]

                # Triangle 1
                points.InsertNextPoint(i, j, z1)
                points.InsertNextPoint(i, (j + 1), z2)
                points.InsertNextPoint((i + 1), j, z3)

                triangle = vtk.vtkTriangle()
                triangle.GetPointIds().SetId(0, count)
                triangle.GetPointIds().SetId(1, count + 1)
                triangle.GetPointIds().SetId(2, count + 2)

                triangles.InsertNextCell(triangle)

                z1 = topography[i][j + 1]
                z2 = topography[i + 1][j + 1]
                z3 = topography[i + 1][j]

                # Triangle 2
                points.InsertNextPoint(i, (j + 1), z1)
                points.InsertNextPoint((i + 1), (j + 1), z2)
                points.InsertNextPoint((i + 1), j, z3)

                triangle = vtk.vtkTriangle()
                triangle.GetPointIds().SetId(0, count + 3)
                triangle.GetPointIds().SetId(1, count + 4)
                triangle.GetPointIds().SetId(2, count + 5)

                count += 6

                triangles.InsertNextCell(triangle)

                lr = (32, 56)
                factor = (abs(z1) / float(self.z_max)) * (lr[1] - lr[0])
                factor = lr[1] - factor
                r = tuple(int(i * 255) for i in colorsys.hls_to_rgb(19.02, factor / 100, 28.67 / 100))
                colors.InsertNextTypedTuple(r)
                colors.InsertNextTypedTuple(r)
                colors.InsertNextTypedTuple(r)
                colors.InsertNextTypedTuple(r)
                colors.InsertNextTypedTuple(r)
                colors.InsertNextTypedTuple(r)

        # Create a polydata object.
        trianglePolyData = vtk.vtkPolyData()

        # Add the geometry and topology to the polydata.
        trianglePolyData.SetPoints(points)
        trianglePolyData.GetPointData().SetScalars(colors)
        trianglePolyData.SetPolys(triangles)

        # Clean the polydata so that the edges are shared!
        cleanPolyData = vtk.vtkCleanPolyData()
        cleanPolyData.SetInputData(trianglePolyData)

        # Use a filter to smooth the data (will add triangles and smooth).
        self.smooth_loop = vtk.vtkLoopSubdivisionFilter()
        self.smooth_loop.SetNumberOfSubdivisions(3)
        self.smooth_loop.SetInputConnection(cleanPolyData.GetOutputPort())

    def draw(self, s: tuple, e: tuple, maxloop=3000, w0=(0, 0, 0)):
        if self.prev_actor is not None:
            self.renderer.RemoveActor(self.prev_actor)

        # Create a spline and add the points
        spline = vtk.vtkParametricSpline()
        spline.SetPoints(self.get_points(s, e, maxloop=maxloop))
        functionSource = vtk.vtkParametricFunctionSource()
        functionSource.SetUResolution(maxloop)
        functionSource.SetParametricFunction(spline)

        # Map the spline
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(functionSource.GetOutputPort())

        # Define the line actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(self.colors.GetColor3d("Red"))
        actor.GetProperty().SetLineWidth(10)

        transform = vtk.vtkTransform()
        transform.Translate(w0)
        actor.SetUserTransform(transform)

        self.prev_actor = actor
        return actor

    def get_points(self, s, e, tolerance=0.001, maxloop=3000):
        locator = vtk.vtkCellLocator()
        locator.SetDataSet(self.smooth_loop.GetOutput())
        locator.BuildLocator()

        # Find Parameters
        s = tuple(x / self.distance_diff for x in s)
        e = tuple(x / self.distance_diff for x in e)
        slope = (s[1] - e[1]) / (s[0] - e[0])
        dist = math.sqrt(vtk.vtkMath.Distance2BetweenPoints(e, s)) / maxloop
        print("dist({}, {}) = {}".format(s, e, dist * maxloop))

        points = vtk.vtkPoints()
        for i in range(maxloop):
            x = i * dist
            y = slope * x
            p1 = [x + e[0], y + e[1], -self.z_max]
            p2 = [x + e[0], y + e[1], self.z_max]

            # Outputs (we need only pos which is the x, y, z position
            # of the intersection)
            t = vtk.mutable(0)
            pos = [0.0, 0.0, 0.0]
            pcoords = [0.0, 0.0, 0.0]
            subId = vtk.mutable(0)
            locator.IntersectWithLine(p1, p2, tolerance, t, pos, pcoords, subId)

            # Error Handling
            if (abs(s[0] - pos[0]) < 0.01) or (abs(s[1] - pos[1]) < 0.01):
                print("Exited at {} step. ({}, {})".format(i, x, y))
                break

            # Add a slight offset in z.
            pos[2] += 0.01
            # Add the x, y, z position of the intersection.
            points.InsertNextPoint(pos)

        return points

    def get_mesh(self, w0=(0, 0, 0)):
        # Create a mapper and actor for smoothed dataset.
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(self.smooth_loop.GetOutputPort())

        mesh_actor = vtk.vtkActor()
        mesh_actor.SetMapper(mapper)
        mesh_actor.GetProperty().SetInterpolationToFlat()

        # Update the pipeline so that vtkCellLocator finds cells!
        self.smooth_loop.Update()

        transform = vtk.vtkTransform()
        transform.Translate(w0)
        mesh_actor.SetUserTransform(transform)

        return mesh_actor


class Text(Object):

    def __init__(self, renderer, colors):
        super().__init__(renderer, colors)

    def draw(self, text, x, y, z):
        actor = vtk.vtkTextActor3D()
        actor.SetInput(text)

        transform = vtk.vtkTransform()
        transform.Translate(x, y, z)
        transform.RotateX(70)
        transform.Scale(0.2, 0.2, 0.2)
        actor.SetUserTransform(transform)

        return actor


class Points(Object):

    def __init__(self, renderer, colors, file):
        super().__init__(renderer, colors)
        self.file = file

    def draw(self, distance_diff=0.5, w0=(0, 0, 0)):
        with open(self.file) as fp:
            fp.readline()
            line = fp.readline()
            while line:
                row = line.strip().split(",")
                title = row[0]
                x = int(float(row[1]) / distance_diff) + w0[0]
                y = int(float(row[2]) / distance_diff) + w0[1]
                z = float(row[3]) / distance_diff + w0[2]

                sphereSource = vtk.vtkSphereSource()
                sphereSource.SetCenter(x, y, z)
                sphereSource.SetRadius(0.4)
                # Make the surface smooth.
                sphereSource.SetPhiResolution(100)
                sphereSource.SetThetaResolution(100)

                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(sphereSource.GetOutputPort())

                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.GetProperty().SetColor(self.colors.GetColor3d("Cornsilk"))

                self.renderer.AddActor(actor)
                self.renderer.AddActor(Text(self.renderer, self.colors).draw(title, x, y, z + 2))

                line = fp.readline()
