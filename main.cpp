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

struct Tri {
    Point a, b, c;

    Tri(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3) {
        a = Point(x1,y1,z1);
        b = Point(x2,y2,z2);
        c = Point(x3,y3,z3);
    }

};

static vector<Tri>* faces;
static double scale = 1;
const double MAX_SIZE = 2;
static bool facesOn = false;
static double xRot = 0, yRot = 0;
static double rotSpeed = 10;
/* GLUT callback Handlers */

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
    for (size_t i = 0; i < tris.size(); ++i) {
        glNormal3f(tris[i].a.x,tris[i].a.y,tris[i].a.z);
        glVertex3f(tris[i].a.x,tris[i].a.y,tris[i].a.z);

        glNormal3f(tris[i].b.x,tris[i].b.y,tris[i].b.z);
        glVertex3f(tris[i].b.x,tris[i].b.y,tris[i].b.z);

        glNormal3f(tris[i].c.x,tris[i].c.y,tris[i].c.z);
        glVertex3f(tris[i].c.x,tris[i].c.y,tris[i].c.z);
    }
}

static void calculateScale(const vector<Tri>& tris) {
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

    if (furthest > MAX_SIZE)
        scale = MAX_SIZE / furthest;
    else
        scale = 1;
}

static void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3d(1,0,0);

     glPushMatrix();
        glTranslated(0,0,-6);
        glRotated(xRot,0,1,0);
        glRotated(yRot,1,0,0);
        glScaled(scale, scale, scale);

        if (facesOn) glBegin(GL_TRIANGLES);
        else glBegin(GL_POINTS);
            renderFaces(*faces);
        glEnd();
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

const GLfloat mat_ambient[]    = { 0.7f, 0.7f, 0.7f, 1.0f };
const GLfloat mat_diffuse[]    = { 0.8f, 0.8f, 0.8f, 1.0f };
const GLfloat mat_specular[]   = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat high_shininess[] = { 25.0f };

/* Program entry point */

int main(int argc, char *argv[])
{
    vector<Tri> t;// { {0,0,0, 10,0,0, 0,1,10}, {0,0,0, 0,-10,0, 0,1,10}};
    t.push_back(Tri(0,0,0, 10,0,0, 0,1,10));
    t.push_back(Tri(0,0,0, 0,-10,0, 0,1,10));
    faces = &t;

    calculateScale(*faces);

    glutInit(&argc, argv);
    glutInitWindowSize(640,640);
    glutInitWindowPosition(10,10);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

    glutCreateWindow("3D Convex Hulls");

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

    glMaterialfv(GL_FRONT, GL_AMBIENT,   mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE,   mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);

    glutMainLoop();

    return EXIT_SUCCESS;
}
