/*
 * 3D Convex Hull Project
 * Patrick Lin, Abe Gellis
 *
 * Some elements derived from GLUT Shapes Demo by Nigel Stewart
 */

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <stdlib.h>
#include <vector>

using namespace std;

struct Point {
    float x,y,z;
    Point(float x = 0, float y = 0, float z = 0) {this->x = x; this->y = y; this->z = z;}
};

struct Line {
    Point a, b;

    Line(float x1, float y1, float z1, float x2, float y2, float z2) {
        a = Point(x1,y1,z1);
        b = Point(x2,y2,z2);
    }
};

struct Tri {
    Point a, b, c;

    Tri(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3) {
        a = Point(x1,y1,z1);
        b = Point(x2,y2,z2);
        c = Point(x3,y3,z3);
    }

};

static vector<Tri>* faces;
static vector<Line>* lines;
static vector<Point>* points;
static double scale = 1;
const double MAX_SIZE = 2;
static bool facesOn = true;
static double xRot = 0, yRot = 0;
static double rotSpeed = 10;
const double POINT_SIZE = 2;
/* GLUT callback Handlers */

// Determines if three points are collinear
// Equation (9) of http://mathworld.wolfram.com/Collinear.html
static bool collinear(const Point& a, const Point& b, const Point& c) {
    return 0 == ((a.x * b.y * c.z) + (b.x * c.y * a.z) + (c.x * a.y * b.z)
                    - (a.x * c.y * b.z) - (b.x * a.y * c.z) - (c.x * b.y * a.z));
}

// Determines if four points are coplanar
// From http://mathworld.wolfram.com/Coplanar.html
static bool coplanar(const Point& a, const Point& b, const Point& c, const Point& d) {
    Point t1(c.x - a.x, c.y - a.y, c.z - a.z); // c - a
    Point t2(b.x - a.x, b.y - a.y, b.z - a.z); // b - a
    Point t3(d.x - c.x, d.y - c.y, d.z - c.z); // d - c

    // t2 x t3
    Point t4(t2.y * t3.z - t2.z * t3.y, t2.z * t3.x - t2.x * t3.z, t2.x * t3.y - t2.y * t3.x);

    // coplanar if 0 == (t1 . (t2 x t3))
    return (0 == (t1.x * t4.x + t1.y * t4.y + t1.z * t4.z));
}

static void resize(int width, int height)
{
    const float ar = (float) width / (float) height;

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-ar, ar, -1.0, 1.0, 2.0, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity() ;
}

static void renderFaces(const vector<Tri>& tris) {
    glEnable(GL_LIGHTING);
    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < tris.size(); ++i) {
        glNormal3f(tris[i].a.x,tris[i].a.y,tris[i].a.z);
        glVertex3f(tris[i].a.x,tris[i].a.y,tris[i].a.z);

        glNormal3f(tris[i].b.x,tris[i].b.y,tris[i].b.z);
        glVertex3f(tris[i].b.x,tris[i].b.y,tris[i].b.z);

        glNormal3f(tris[i].c.x,tris[i].c.y,tris[i].c.z);
        glVertex3f(tris[i].c.x,tris[i].c.y,tris[i].c.z);
    }
    glEnd();
}

static void renderLines(const vector<Line>& lines) {
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    for (size_t i = 0; i < lines.size(); ++i) {
        glVertex3f(lines[i].a.x,lines[i].a.y,lines[i].a.z);
        glVertex3f(lines[i].b.x,lines[i].b.y,lines[i].b.z);
    }
    glEnd();
}

static void renderPoints(const vector<Point>& points) {
    glDisable(GL_LIGHTING);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < points.size(); ++i) {
        glVertex3f(points[i].x,points[i].y,points[i].z);
    }
    glEnd();
}

static void calculateScale(const vector<Tri>& tris, const vector<Line>& lines, const vector<Point>& points) {
    double furthest = 0;

    for (size_t i = 0; i < tris.size(); ++i) {
        if (tris[i].a.x > furthest) furthest = tris[i].a.x;
        if (tris[i].a.y > furthest) furthest = tris[i].a.y;
        if (tris[i].a.z > furthest) furthest = tris[i].a.z;
        if (tris[i].b.x > furthest) furthest = tris[i].b.x;
        if (tris[i].b.y > furthest) furthest = tris[i].b.y;
        if (tris[i].b.z > furthest) furthest = tris[i].b.z;
        if (tris[i].c.x > furthest) furthest = tris[i].c.x;
        if (tris[i].c.y > furthest) furthest = tris[i].c.y;
        if (tris[i].c.z > furthest) furthest = tris[i].c.z;
    }

    for (size_t i = 0; i < lines.size(); ++i) {
        if (lines[i].a.x > furthest) furthest = lines[i].a.x;
        if (lines[i].a.y > furthest) furthest = lines[i].a.y;
        if (lines[i].a.z > furthest) furthest = lines[i].a.z;
        if (lines[i].b.x > furthest) furthest = lines[i].b.x;
        if (lines[i].b.y > furthest) furthest = lines[i].b.y;
        if (lines[i].b.z > furthest) furthest = lines[i].b.z;
    }

    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i].x > furthest) furthest = points[i].x;
        if (points[i].y > furthest) furthest = points[i].y;
        if (points[i].z > furthest) furthest = points[i].z;
    }

    if (furthest > MAX_SIZE)
        scale = MAX_SIZE / furthest;
    else
        scale = 1;
}

static void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

     glPushMatrix();
        glTranslated(0,0,-6);
        glRotated(xRot,0,1,0);
        glRotated(yRot,1,0,0);
        glScaled(scale, scale, scale);

        glColor4d(1,0,0,1);
        renderLines(*lines);

        renderPoints(*points);

        glColor4d(1,0,0,.5);
        if (facesOn)
            renderFaces(*faces);


    glPopMatrix();

    glutSwapBuffers();
}


static void key(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27 :
        case 'q':
            exit(0);
            break;
        case 'f':
            facesOn = !facesOn;
            break;
        case 'a':
            xRot -= rotSpeed;
            break;
        case 'd':
            xRot += rotSpeed;
            break;
        case 'w':
            yRot -= rotSpeed;
            break;
        case 's':
            yRot += rotSpeed;
            break;

    }

    glutPostRedisplay();
}

static void idle(void)
{
    glutPostRedisplay();
}

const GLfloat light_ambient[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
const GLfloat light_diffuse[]  = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_position[] = { 2.0f, 5.0f, 5.0f, 0.0f };

const GLfloat mat_ambient[]    = { 0.7f, 0.7f, 0.7f, 0.5f };
const GLfloat mat_diffuse[]    = { 0.8f, 0.8f, 0.8f, 0.5f };
const GLfloat mat_specular[]   = { 1.0f, 1.0f, 1.0f, 0.5f };
const GLfloat high_shininess[] = { 25.0f };

/* Program entry point */

int main(int argc, char *argv[])
{
    vector<Tri> t;// { {0,0,0, 10,0,0, 0,1,10}, {0,0,0, 0,-10,0, 0,1,10}};
    t.push_back(Tri(0,0,0, 10,0,0, 0,1,10));
    t.push_back(Tri(0,0,0, 0,-10,0, 0,1,10));

    vector<Line> l;
    l.push_back(Line(5,5,5,-5,-5,5));

    vector<Point> p;
    p.push_back(Point(3,3,3));


    faces = &t;
    lines = &l;
    points = &p;

    calculateScale(*faces, *lines, *points);

    glutInit(&argc, argv);
    glutInitWindowSize(640,640);
    glutInitWindowPosition(10,10);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

    glutCreateWindow("3D Convex Hulls");

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable( GL_BLEND );

    glutReshapeFunc(resize);
    glutDisplayFunc(display);
    glutKeyboardFunc(key);
    glutIdleFunc(idle);

    glClearColor(0,0,0,1);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);

    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    /*glMaterialfv(GL_FRONT, GL_AMBIENT,   mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE,   mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);*/

    glPointSize(POINT_SIZE);

    glutMainLoop();

    return EXIT_SUCCESS;
}
