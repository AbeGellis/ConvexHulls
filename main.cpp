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
#endif // __APPLE__

#ifdef _WIN32
#include <windows.h>
#else
#include <chrono>
#include <thread>
#endif

#include <stdlib.h>
#include <vector>
#include <queue>
#include <iostream>
#include <fstream>

#include <math.h>
#include <unistd.h>

using namespace std;

struct Color {
    float r,g,b,a;
    Color(float r = 0, float g = 0, float b = 0, float a = 0) : r(r), g(g), b(b), a(a) {}
};


static Color randomColor(float alpha = .5) {
    return Color((float) (rand() % 1000) / 1000, (float) (rand() % 1000) / 1000, (float) (rand() % 1000) / 1000, alpha);
}

struct Point {
    int index;
    float x,y,z;
    Color col;
    bool assigned; // For use with Quickhull
    Point(float x = 0, float y = 0, float z = 0, int index = -1, Color c = Color(1,0,0,1)) : index(index), x(x), y(y), z(z), col(c), assigned(false) {}
};
bool operator==(const Point& lhs, const Point& rhs) {
    return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}
bool operator!=(const Point& lhs, const Point& rhs) {
    return !(lhs == rhs);
}

struct Line {
    Point a, b;
    Color col;

    Line(Point a, Point b, Color c = Color(1,0,0,.5)) : a(a), b(b), col(c) {}
};

struct Tri {
    Point a, b, c;
    Color col;
    vector<Point*> outside_set; // For use with Quickhull
    vector<Tri*> adjacent; // For use with Quickhull
    bool vis; // For use with Quickhull
    bool del; // For use with Quickhull
    Tri(Point a, Point b, Point c, Color col = randomColor()) : a(a), b(b), c(c), col(col), vis(false), del(false) {}
};
bool operator==(const Tri& lhs, const Tri& rhs) {
    return (lhs.a == rhs.a && lhs.b == rhs.b && lhs.c == rhs.c);
}
bool operator!=(const Tri& lhs, const Tri& rhs) {
    return !(lhs == rhs);
}

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

static void renderFaces(const vector<Tri>& tris) {
    glEnable(GL_LIGHTING);
    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < tris.size(); ++i) {
        if ( tris[i].del )
            continue;
        glColor4f(tris[i].col.r, tris[i].col.g, tris[i].col.b, tris[i].col.a);
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
        glColor4f(lines[i].col.r, lines[i].col.g, lines[i].col.b, lines[i].col.a);
        glVertex3f(lines[i].a.x,lines[i].a.y,lines[i].a.z);
        glVertex3f(lines[i].b.x,lines[i].b.y,lines[i].b.z);
    }
    glEnd();
}

static void renderPoints(const vector<Point>& points) {
    glDisable(GL_LIGHTING);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < points.size(); ++i) {
        glColor4f(points[i].col.r, points[i].col.g, points[i].col.b, points[i].col.a);
        glVertex3f(points[i].x,points[i].y,points[i].z);
    }
    glEnd();
}

static void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

     glPushMatrix();
        glTranslated(0,0,-6);
        glRotated(xRot,0,1,0);
        glRotated(yRot,1,0,0);
        glScaled(scale, scale, scale);

        renderLines(*lines);

        renderPoints(*points);


            renderFaces(*faces);


    glPopMatrix();

    glutSwapBuffers();
}


static float dotProd(const Point& t1, const Point& t2) {
    return t1.x * t2.x + t1.y * t2.y + t1.z * t2.z;
}

static Point crossProd(const Point& t1, const Point& t2) {
    return Point(t1.y * t2.z - t1.z * t2.y, t1.z * t2.x - t1.x * t2.z, t1.x * t2.y - t1.y * t2.x);
}

// Determines if three points are collinear
// Equation (5) of http://mathworld.wolfram.com/Collinear.html
static bool collinear(const Point& a, const Point& b, const Point& c) {
    Point t1(b.x - a.x, b.y - a.y, b.z - a.z); // b - a
    Point t2(a.x - c.x, a.y - c.y, a.z - c.z); // a - c

    // t1 x t2
    Point t3 = crossProd(t1, t2);

    return (0 == dotProd(t3, t3));
}

// Determines if four points are coplanar
// From http://mathworld.wolfram.com/Coplanar.html
static bool coplanar(const Point& a, const Point& b, const Point& c, const Point& d) {
    Point t1(c.x - a.x, c.y - a.y, c.z - a.z); // c - a
    Point t2(b.x - a.x, b.y - a.y, b.z - a.z); // b - a
    Point t3(d.x - c.x, d.y - c.y, d.z - c.z); // d - c

    // coplanar if 0 == (t1 . (t2 x t3))
    return (0 == dotProd(t1, crossProd(t2, t3)));
}

static bool nondegenerate(const vector<Point>& p) {
    for ( int i = 0; i < p.size(); ++i ) {
        for ( int j = i + 1; j < p.size(); ++j ) {
            for ( int k = j + 1; k < p.size(); ++k ) {
                if ( collinear(p[i], p[j], p[k]) ) {
                    cerr << "3 Collinear points detected!\n";
                    return false;
                }
                for ( int l = k + 1; l < p.size(); ++l ) {
                    if ( coplanar(p[i], p[j], p[k], p[l]) ) {
                        cerr << "4 Coplanar points detected!\n";
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

// From O'Rourke, Computational Geometry in C, Code 4.16
// if a point is "outside" then vSign == -1
static int vSign( const Tri& t, const Point& p ) {
    Point t1(t.a.x - p.x, t.a.y - p.y, t.a.z - p.z);
    Point t2(t.b.x - p.x, t.b.y - p.y, t.b.z - p.z);
    Point t3(t.c.x - p.x, t.c.y - p.y, t.c.z - p.z);

    // vSign = sign(t1 . (t2 x t3))
    float vol = dotProd(t1, crossProd(t2, t3));
    return vol > 0.01 ? 1 : vol < -0.01 ? -1 : 0;
}

// Distance from point to the plane described by a triangle
// Equation (12,13) of http://mathworld.wolfram.com/Point-PlaneDistance.html
static float distPointToTri( const Point& p, const Tri& t ) {
    Point t1(t.b.x - t.a.x, t.b.y - t.a.y, t.b.z - t.a.z); // b - a
    Point t2(t.c.x - t.a.x, t.c.y - t.a.y, t.c.z - t.a.z); // c - a

    Point n = crossProd(t1, t2);
    float nSize = sqrt(dotProd(n, n));
    Point nhat(n.x / nSize, n.y / nSize, n.z / nSize);

    Point t3(p.x - t.a.x, p.y - t.a.y, p.z - t.a.y); // p - a
    return dotProd(nhat, t3);
}


static void sleep(int millis) {
#ifdef _WIN32
    Sleep(millis);
#else
    this_thread::sleep_for(chrono::milliseconds(200));
#endif
}

// 3D Quickhull Algorithm
static void quickhull(vector<Point> p, vector<Tri>& t) { // Passes a COPY of the points. Intentional.
    t.clear();
    sleep(200);
    display();
    cout << "Assignment OK\n";
    // create simplex of 4 points
    int perm[4][4] = { { 0, 1, 2, 3 }, { 0, 1, 3, 2 }, { 0, 2, 3, 1 }, { 1, 2, 3, 0 } };
    for ( int i = 0; i < 4; ++i ) {
        Tri f(p[perm[i][0]], p[perm[i][1]], p[perm[i][2]]);
        if ( !p[perm[i][3]].assigned && vSign(f, p[perm[i][3]]) < 0 ) { // Make the orientations correct: any other point
                                            // on the hull should be "inside"
            Point temp = f.c;
            f.c = f.b;
            f.b = temp;
        }
        t.push_back(f);
        sleep(200);
        display();
    }
    for ( int i = 0; i < 4; ++i ) { // Add adjacent pointers
        for ( int j = 0; j < 4; ++j ) {
            t[i].adjacent.push_back(&(t[j]));
        }
    }

    cout << "Tet OK\n";

    queue<Tri*> t_to_process;
    // for each facet F, for each unassigned point p, if p is above F, assign p to F's outside set
    //      if F has outside points, we will process it later...
    for ( int i = 0; i < 4; ++i ) {
        for ( int j = 0; j < p.size(); ++j ) {
            if ( vSign(t[i], p[j]) < 0 ) {
                t[i].outside_set.push_back(&(p[j]));
                p[j].assigned = true;
            }
        }
        if ( t[i].outside_set.size() > 0 ) {
            t_to_process.push(&(t[i]));
        }
    }

    cout << "Outside set OK\n";

    int round = 0;

    // for each facet F with a non-empty outside set
    while ( t_to_process.size() > 0 ) {
        cout << "round: " << round << '\n';
        round++;
        Tri *f = t_to_process.front();
        t_to_process.pop();

        if ( f->del )
            continue;

        cout << "Pop OK\n";

        // select the furthest point p of F's outside set
        int argmax = -1;
        int max = 0;
        cout << "Outside size: " << f->outside_set.size() << '\n';
        for ( int j = 0; j < f->outside_set.size(); ++j ) {
            cout << f->outside_set[j]->index << '\n';
            int dist = distPointToTri( *(f->outside_set[j]), *f );
            if ( dist > max ) {
                max = dist;
                argmax = j;
            }
        }

        int argmax_real_index = f->outside_set[argmax]->index;

        cout << "point: " << argmax_real_index << '\n';

        points->at(argmax_real_index).col.g = 1;
        sleep(200);
        display();


        // initialize the visible set V to F
        // for all neighbors N of facets to F
        for ( int j = 0; j < f->adjacent.size(); ++j ) {
            if ( f->adjacent[j]->del ) // Ignore deleted facets
                continue;
            // if p is above N, add N to V
            if ( vSign( *(f->adjacent[j]), *(f->outside_set[argmax]) ) < 0 ) {
                f->adjacent[j]->vis = true;
                f->adjacent[j]->col.r = 0;
            }
        }
        sleep(200);
        display();

        // the boundary of V is the set of horizon ridges H
        // for each ridge R in H
        for ( int i = 0; i < p.size(); ++i ) { // This probably could have been implemented
                                                // more efficiently using proper data structure
            for ( int j = 0; j < p.size(); ++j ) {

                int vis_count = 0;
                for ( int k = 0; k < t.size(); ++k ) {
                    if ( t[k].del ) // Ignore deleted facets
                        continue;
                    if ( t[k].vis && ((t[k].a == p[i] && t[k].b == p[j])
                        || (t[k].a == p[i] && t[k].c == p[j])
                        || (t[k].b == p[i] && t[k].c == p[j])
                        || (t[k].a == p[j] && t[k].b == p[i])
                        || (t[k].a == p[j] && t[k].c == p[i])
                        || (t[k].b == p[j] && t[k].c == p[i])
                        )) {
                        vis_count++;

                        if ( vis_count >= 2 )
                            break;
                    }
                }

                if ( vis_count == 1) {
                    // create a new facet F' from R and p
                    Tri newF(p[i],p[j],p[argmax_real_index]);
                    for ( int k = 0; k < t.size(); ++k ) {// Make the orientations correct: any other point
                                                            // on the hull should be "inside"
                        if ( t[k].a != p[i] && t[k].a != p[j] ) {
                            if ( vSign(newF, t[k].a) < 0 ) {
                                Point temp = newF.a;
                                newF.a = newF.b;
                                newF.b = temp;
                                break;
                            }
                        }
                        if ( t[k].b != p[i] && t[k].b != p[j] ) {
                            if ( vSign(newF, t[k].b) < 0 ) {
                                Point temp = newF.a;
                                newF.a = newF.b;
                                newF.b = temp;
                                break;
                            }
                        }
                        if ( t[k].c != p[i] && t[k].c != p[j] ) {
                            if ( vSign(newF, t[k].c) < 0 ) {
                                Point temp = newF.a;
                                newF.a = newF.b;
                                newF.b = temp;
                                break;
                            }
                        }
                    }
                    t.push_back(newF);
                    // Link the new facet to its neighbors
                    for ( int k = 0; k < t.size(); ++k ) {
                        if ( t[k].del ) // Ignore deleted facets
                            continue;
                         if ( (t[k].a == newF.a && t[k].b == newF.b)
                            || (t[k].a == newF.a && t[k].c == newF.b)
                            || (t[k].b == newF.a && t[k].c == newF.b)
                            || (t[k].a == newF.b && t[k].b == newF.a)
                            || (t[k].a == newF.b && t[k].c == newF.a)
                            || (t[k].b == newF.b && t[k].c == newF.a)
                            || (t[k].a == newF.a && t[k].b == newF.c)
                            || (t[k].a == newF.a && t[k].c == newF.c)
                            || (t[k].b == newF.a && t[k].c == newF.c)
                            || (t[k].a == newF.c && t[k].b == newF.a)
                            || (t[k].a == newF.c && t[k].c == newF.a)
                            || (t[k].b == newF.c && t[k].c == newF.a)
                            || (t[k].a == newF.b && t[k].b == newF.c)
                            || (t[k].a == newF.b && t[k].c == newF.c)
                            || (t[k].b == newF.b && t[k].c == newF.c)
                            || (t[k].a == newF.c && t[k].b == newF.b)
                            || (t[k].a == newF.c && t[k].c == newF.b)
                            || (t[k].b == newF.c && t[k].c == newF.b)
                            ) {
                            t.back().adjacent.push_back(&(t[k]));
                        }
                    }
                    t.back().adjacent.push_back(&(t.back()));

                    // For each point q in an outside set of a facet of V
                    for ( int j = 0; j < p.size(); ++j ) {
                        // if q is above F' assing q to F''s outside set
                        if ( !p[j].assigned && vSign(t.back(), p[j]) < 0 ) {
                            t.back().outside_set.push_back(&(p[j]));
                            p[j].assigned = true;
                        }
                    }

                    cout << "Outside size: " << t.back().outside_set.size() << '\n';

                    if ( t.back().outside_set.size() > 0 ) {
                        t_to_process.push(&(t.back()));
                        cout << "Push!\n";
                    }

                    sleep(200);
                    display();



                }
            }
        }
        cout << "Ready to delete\n";

        // Delete the facets in V
        for ( int j = 0; j < f->adjacent.size(); ++j ) {
            if ( f->adjacent[j]->vis ) {// Ignore deleted facets

                f->adjacent[j]->del = true;
            }
        }

        points->at(argmax_real_index).col.g = 0;
        sleep(200);
        display();
    }
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

static void key(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27 :
            exit(0);
            break;
        case 'q':
            quickhull(*points, *faces);
            break;
        case 'c':
            faces->clear();
            for ( int i = 0; i < points->size(); ++i ) {
                points->at(i).col = Color(1,0,0,1);
            }
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
    char* file_name;
    bool using_file = false;
    for ( int i = 0; i < argc; ++i ) {
        if ( strcmp(argv[i], "--pointset") == 0 ) {
            file_name = argv[i+1];
            using_file = true;
            break;
        }
    }

    vector<Point> p;

    long long N;
    if ( using_file ) {
        ifstream ifs;
        ifs.open(file_name);
        if (!ifs) {
            cerr << "Error: Could not open file " << file_name << "\n";
            return EXIT_FAILURE;
        }
        long long index = 0;
        float x, y, z;
        while ( ifs >> x ) {
            ifs >> y;
            ifs >> z;
            p.push_back(Point(x,y,z,index));
            ++index;
        }

        if ( p.size() < 4 ) {
            cerr << "Error: Cannot find the 3D convex hull of fewer than four points.\n";
            return EXIT_FAILURE;
        }
        N = index;
    } else {
        cout << "How many points will you be entering? ";
        cin >> N;
        if ( N < 4 ) {
            cerr << "Error: Cannot find the 3D convex hull of fewer than four points.\n";
            return EXIT_FAILURE;
        }
        cout << "Please enter your points in the following format:\nx y z\n\n";

        float x, y, z;
        for ( int i = 0; i < N; ++i ) {
            cout << "Point #" << i+1 << ": ";
            cin >> x >> y >> z;
            p.push_back(Point(x,y,z,i));
        }
    }

    if ( !nondegenerate(p) ) {
        cerr << "Point set must be nondegenerate (no 3 points can be collinear, no 4 points coplanar).\n";
        return EXIT_FAILURE;
    }



    vector<Tri> t;// { {0,0,0, 10,0,0, 0,1,10}, {0,0,0, 0,-10,0, 0,1,10}};
    t.reserve(N * N * N);
//    t.push_back(Tri(0,0,0, 10,0,0, 0,1,10));
//    t.push_back(Tri(0,0,0, 0,-10,0, 0,1,10));

    vector<Line> l;
//    l.push_back(Line(5,5,5,-5,-5,5));


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
    // glutIdleFunc(idle);

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
