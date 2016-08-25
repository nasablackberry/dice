# pyODE example 3: Collision detection

# Originally by Matthias Baas.
# Updated by Pierre Gay to work without pygame or cgkit.

import sys, os, random, time
from threading import Timer, Thread, Event

from math import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PySide.QtCore import *
from PySide.QtGui import *
from PySide.QtOpenGL import *

import ode


# geometric utility functions
def scalp(vec, scal):
    vec[0] *= scal
    vec[1] *= scal
    vec[2] *= scal


def length(vec):
    return sqrt(vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2)


def override(interface_class):
    """
    Method to implement Java-like derived class method override annotation.
    Courtesy of mkorpela's answer at
    http://stackoverflow.com/questions/1167617/in-python-how-do-i-indicate-im-overriding-a-method
    """

    def override(method):
        assert (method.__name__ in dir(interface_class))
        return method

    return override


class DiceWidget(QGLWidget):
    "GUI rectangle that displays a teapot"

    # set by experiment
    POSX = 0
    POSY = 5.0
    POSZ = 0
    FORCEX = 1000
    FORCEY = 0
    FORCEZ = 0
    PIMUL = 1.1
    FRAMEDIFF = 20

    world = None

    # A joint group for the contact joints that are generated whenever
    # two bodies collide
    contactgroup = None

    # A list with ODE bodies
    bodies = []

    # The geoms for each of the bodies
    geoms = []

    # Some variables used inside the simulation loop
    fps = 60
    dt = 1.0 / fps
    running = True
    state = 0
    counter = 0
    objcount = 0
    lasttime = time.time()

    # default
    num_dice = 0

    def __init__(self, num_dice, *args, **kwargs):
        super(DiceWidget, self).__init__(*args, **kwargs)
        self.num_dice = num_dice

        # Create a world object
        self.world = ode.World()
        self.world.setGravity((0, -9.81, 0))
        self.world.setERP(0.8)
        self.world.setCFM(1E-5)

        # Create a space object
        self.space = ode.Space()

        self.floor = ode.GeomPlane(self.space, (0, 1, 0), 0)
        # self.roof = ode.GeomPlane(self.space, (0, -1, 0), -self.height())
        # self.wall_left = ode.GeomPlane(self.space, (-1, 0, 0), self.width())
        # self.wall_left = ode.GeomPlane(self.space, (-1, 0, 0), 0)
        self.wall_top = ode.GeomPlane(self.space, (1, 1, 0), -self.width() / 2)
        self.wall_right = ode.GeomPlane(self.space, (-1, -1, 0), -self.width() / 2)

        self.contactgroup = ode.JointGroup()

    @override(QGLWidget)
    def initializeGL(self):
        """runs once, after OpenGL context is created"""
        glEnable(GL_DEPTH_TEST)
        glClearColor(1, 1, 1, 0)  # white background
        glShadeModel(GL_SMOOTH)
        glEnable(GL_COLOR_MATERIAL)
        glMaterialfv(GL_FRONT, GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
        glMaterialfv(GL_FRONT, GL_SHININESS, [50.0])
        glLightfv(GL_LIGHT0, GL_POSITION, [1.0, 1.0, 1.0, 0.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [1.0, 1.0, 1.0, 0.0])
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        self.orient_camera()
        gluLookAt(0, 0, -10,  # camera
                  0, 0, 0,  # focus
                  0, 1, 0)  # up vector
        self.prepare_gl()
        timer = QTimer(self)
        self.connect(timer, SIGNAL("timeout()"), self.animate)
        timer.start(20)

    # prepare_GL
    @staticmethod
    def prepare_gl():
        """Prepare drawing.
        """

        # Initialize
        glEnable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glEnable(GL_LIGHTING)
        glEnable(GL_NORMALIZE)
        glShadeModel(GL_FLAT)

        # Projection
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 1.3333, 0.2, 20)

        # Initialize ModelView matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Light source
        glLightfv(GL_LIGHT0, GL_POSITION, [0, 0, 1, 0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1, 1, 1, 1])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [1, 1, 1, 1])
        glEnable(GL_LIGHT0)

        # View transformation
        gluLookAt(0.5, 3.6, 4.8, 0.5, 0.5, 0, 0, 1, 0)

    @override(QGLWidget)
    def paintGL(self):
        """runs every time an image update is needed"""
        glClearColor(0.8, 0.8, 0.9, 0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.draw_dice()

    @override(QGLWidget)
    def resizeGL(self, w, h):
        """runs every time the window changes size"""
        glViewport(0, 0, w, h)
        self.orient_camera()

    def orient_camera(self):
        """update projection matrix, especially when aspect ratio changes"""
        glPushAttrib(GL_TRANSFORM_BIT)  # remember current GL_MATRIX_MODE
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60.0, self.width() / float(self.height()), 1.0, 10.0)
        glPopAttrib()  # restore GL_MATRIX_MODE

    def draw_dice(self):
        self.prepare_gl()
        for b in self.bodies:
            self.draw_body(b)

            # self.swapBuffers()
            # glutSwapBuffers()

    def draw_body(self, body):
        """Draw an ODE body.
        """

        x, y, z = body.getPosition()
        rotation = body.getRotation()
        rot = [rotation[0], rotation[3], rotation[6], 0.,
               rotation[1], rotation[4], rotation[7], 0.,
               rotation[2], rotation[5], rotation[8], 0.,
               x, y, z, 1.0]
        glPushMatrix()
        glMultMatrixd(rot)
        if body.shape == "box":
            sx, sy, sz = body.boxsize
            glScalef(sx, sy, sz)
            glutWireCube(1)
        glPopMatrix()
        glPushMatrix()
        glScalef(0.002, 0.002, 0)
        glTranslatef(-1000, 1200, 0)
        # for c in "origin: %s %s %s (az sx dc)" % (self.POSX, self.POSY, self.POSZ):
        #     glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
        glPopMatrix()
        glPushMatrix()
        glScalef(0.002, 0.002, 0)
        glTranslatef(-1000, 1000, 0)
        # for c in "throw: %s %s %s (fv gb hn)" % (self.FORCEX, self.FORCEY, self.FORCEZ):
        #     glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
        glPopMatrix()
        glPushMatrix()
        glScalef(0.002, 0.002, 0)
        glTranslatef(-1000, 800, 0)
        # for c in "spin: %s (jm)" % self.PIMUL:
        #     glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
        glPopMatrix()
        glPushMatrix()
        glScalef(0.002, 0.002, 0)
        glTranslatef(-1000, 600, 0)
        # for c in "dice gap: %s (k,)" % self.FRAMEDIFF:
        #     glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
        glPopMatrix()

    def animate(self):

        t = self.dt - (time.time() - self.lasttime)
        if t > 0:
            time.sleep(t)

        self.counter += 1

        if self.state == 0:
            if self.counter == self.FRAMEDIFF:
                self.drop_object()
            if self.objcount == self.num_dice:
                # state=1
                self.counter = 0
        # State 1: Explosion and pulling back the objects
        elif self.state == 1:
            if self.counter == 100:
                self.explosion()
            if self.counter > 300:
                self.pull()
            if self.counter == 500:
                self.counter = 20

        self.update()

        # Simulate
        n = 2

        for i in range(n):
            # Detect collisions and create contact joints
            self.space.collide((self.world, self.contactgroup), self.near_callback)

            # Simulation step
            self.world.step(self.dt / n)

            # Remove all contact joints
            self.contactgroup.empty()

        self.lasttime = time.time()

    # drop_object
    def drop_object(self):
        """Drop an object into the scene."""

        body, geom = self.create_box(self.world, self.space, 1000, 0.2, 0.2, 0.2)
        body.setPosition((self.POSX, self.POSY, self.POSZ))
        theta = pi * self.PIMUL
        ct = cos(theta)
        st = sin(theta)
        body.setRotation([ct, 0., -st, 0., 1., 0., st, 0., ct])
        body.addForce([self.FORCEX, self.FORCEY, self.FORCEZ])
        self.bodies.append(body)
        self.geoms.append(geom)
        self.counter = 0
        self.objcount += 1
        # bodies = bodies[-2:]
        # geoms = geoms[-2:]
        print(self.POSX, self.POSY, self.POSZ, self.FORCEX, self.FORCEY, self.FORCEZ, self.PIMUL, self.FRAMEDIFF, "\r")
        sys.stdout.flush()

    @staticmethod
    def create_box(world, space, density, lx, ly, lz):
        """Create a box body and its corresponding geom."""

        # Create body
        body = ode.Body(world)
        mass = ode.Mass()
        mass.setBox(density, lx, ly, lz)
        body.setMass(mass)

        # Set parameters for drawing the body
        body.shape = "box"
        body.boxsize = (lx, ly, lz)

        # Create a box geom for collision detection
        geom = ode.GeomBox(space, lengths=body.boxsize)
        geom.setBody(body)

        return body, geom

    # explosion
    def explosion(self):
        """Simulate an explosion.

        Every object is pushed away from the origin.
        The force is dependent on the objects distance from the origin.
        """

        for b in self.bodies:
            l = b.getPosition()
            d = length(l)
            a = max(0, 40000 * (1.0 - 0.2 * d * d))
            l = [l[0] / 4, l[1], l[2] / 4]
            scalp(l, a / length(l))
            b.addForce(l)

    # pull
    def pull(self):
        """Pull the objects back to the origin.

        Every object will be pulled back to the origin.
        Every couple of frames there'll be a thrust upwards so that
        the objects won't stick to the ground all the time.
        """

        for b in self.bodies:
            l = list(b.getPosition())
            scalp(l, -1000 / length(l))
            b.addForce(l)
            if self.counter % 60 == 0:
                b.addForce((0, 10000, 0))

    # Collision callback
    @staticmethod
    def near_callback(args, geom1, geom2):
        """Callback function for the collide() method.

        This function checks if the given geoms do collide and
        creates contact joints if they do.
        """

        # Check if the objects do collide
        contacts = ode.collide(geom1, geom2)

        # Create contact joints
        world, contactgroup = args
        for c in contacts:
            c.setBounce(0.2)
            c.setMu(5000)
            j = ode.ContactJoint(world, contactgroup, c)
            j.attach(geom1.getBody(), geom2.getBody())

    def keyPressEvent(self, e):
        key = e.getKey()

        if key == "q":
            print()
            sys.exit(0)
        elif key == "a":
            self.POSX -= 0.5
        elif key == "z":
            self.POSX += 0.5
        elif key == "s":
            self.POSY -= 0.5
        elif key == "x":
            self.POSY += 0.5
        elif key == "d":
            self.POSZ -= 0.5
        elif key == "c":
            self.POSZ += 0.5
        elif key == "f":
            self.FORCEX -= 100
        elif key == "v":
            self.FORCEX += 100
        elif key == "g":
            self.FORCEY -= 100
        elif key == "b":
            self.FORCEY += 100
        elif key == "h":
            self.FORCEZ -= 100
        elif key == "n":
            self.FORCEZ += 100
        elif key == "j":
            self.PIMUL -= 0.1
        elif key == "m":
            self.PIMUL += 0.1
        elif key == "k":
            self.FRAMEDIFF -= 1
        elif key == ",":
            self.FRAMEDIFF += 1

        self.objcount = 0
        self.bodies = []
        self.geoms = []


class App(QApplication):

    width = 1024
    height = 768

    def __init__(self, num_dice):
        QApplication.__init__(self, sys.argv)
        self.setApplicationName("Dice Simulation")
        self.mainWindow = QMainWindow()
        self.gl_widget = DiceWidget(num_dice)
        self.mainWindow.setCentralWidget(self.gl_widget)
        self.mainWindow.resize(self.width, self.height)
        self.mainWindow.show()
        sys.exit(self.exec_())  # Start Qt main loop


if __name__ == "__main__":
    App(6)
