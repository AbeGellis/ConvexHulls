/*
 * 3D Convex Hull Project
 * Patrick Lin, Abe Gellis
 *
 * Some visualization elements derived from GLUT Shapes Demo by Nigel Stewart
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
#include <set>
#include <queue>
#include <iostream>
#include <fstream>
#include <algorithm>    //Used for one convenience sort for Tri points

#include <cmath>
#include <unistd.h>

using namespace std;

struct Color {
    float r,g,b,a;
    Color(float r = 0, float g = 0, float b = 0, float a = 0) : r(r), g(g), b(b), a(a) {}
};


static Color randomColor(float alpha = .5) {
    return Color((float) (rand() % 700 + 300) / 1000, (float) (rand() % 600) / 1000, (float) (rand() % 600) / 1000, alpha);
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
//For use in constructing a line set in Giftwrapping
bool operator<(const Point& lhs, const Point& rhs) {
    if (lhs.x == rhs.x) {
        if (lhs.y == rhs.y) {
            if (lhs.z == rhs.z)
                return false;
            else return (lhs.z < rhs.z);
        }
        else return (lhs.y < rhs.y);
    }
    else return (lhs.x < rhs.x);
}

struct Line {
    Point a, b;
    Color col;

    Line(Point a, Point b, Color c = Color(1,0,0,.5)) : a(a), b(b), col(c) {}
};
//For use in constructing a line set in Giftwrapping
//Ignores order of vertices when comparing
bool operator<(const Line& lhs, const Line& rhs) {
    if (max(lhs.a,lhs.b) == max(rhs.a, rhs.b))
        return min(lhs.a,lhs.b) < min(rhs.a, rhs.b);
    else return max(lhs.a,lhs.b) < max(rhs.a, rhs.b);
}

bool operator==(const Line& lhs, const Line& rhs) {
    return (lhs.a == rhs.a && lhs.b == rhs.b);
}
bool operator!=(const Line& lhs, const Line& rhs) {
    return !(lhs == rhs);
}

struct Tri {
    Point a, b, c;
    Color col;
    set<Point*> outside_set; // For use with Quickhull
    set<Tri*> adjacent; // For use with Quickhull
    bool vis; // For use with Quickhull
    bool del; // For use with Quickhull
    Tri(Point a, Point b, Point c, Color col = randomColor()) : a(a), b(b), c(c), col(col), vis(false), del(false) {}

    // Returns the oriented boundary edges
    // Viewed from the "front" the boundary is counterclockwise
    vector<Line> getHalfEdges() {
        vector<Line> v;
        v.push_back(Line(a,b));
        v.push_back(Line(b,c));
        v.push_back(Line(c,a));
        return v;
    }

    // Returns the reversed oriented boundary edges
    // Viewed from the "front" the boundary is counterclockwise
    vector<Line> getReverseHalfEdges() {
        vector<Line> v;
        v.push_back(Line(a,c));
        v.push_back(Line(c,b));
        v.push_back(Line(b,a));
        return v;
    }

    vector<Point> getPointSet() {
        vector<Point> v;
        v.push_back(a);
        v.push_back(b);
        v.push_back(c);
        return v;
    }
};
bool operator==(const Tri& lhs, const Tri& rhs) {
    return (lhs.a == rhs.a && lhs.b == rhs.b && lhs.c == rhs.c);
}
bool operator!=(const Tri& lhs, const Tri& rhs) {
    return !(lhs == rhs);
}
//For use in constructing a Tri set in Giftwrapping
//Ignores order of vertices when comparing
bool operator<(const Tri& lhs, const Tri& rhs) {
    Point leftpoints[] = {lhs.a, lhs.b, lhs.c}, rightpoints[] = {rhs.a, rhs.b, rhs.c};
    sort(leftpoints, leftpoints + 3);
    sort(rightpoints, rightpoints + 3);
    if (leftpoints[2] == rightpoints[2]) {
        if (leftpoints[1] == rightpoints[1]) {
            return leftpoints[0] < rightpoints[0];
        }
        else return leftpoints[1] < rightpoints[1];
    }
    else return leftpoints[2] < rightpoints[2];
}



static vector<Tri>* faces;
static vector<Line>* lines;
static vector<Point>* points;
static double scale = 1;
static int bg = 0;
const double DISPLAY_SIZE = .6;
static bool facesOn = true;
static double rotSpeed = 10;
static double POINT_SIZE = 2;
static int SLEEP_TIME = 500;
static double DEG_TO_RAD = 3.141592653589793 / 180.0;
static float rotMat[] = { 1, 0, 0, 0,
                          0, 1, 0, 0,
                          0, 0, 1, 0,
                          0, 0, 0, 1 };

/* GLUT callback Handlers */

static void renderFaces(const vector<Tri>& tris) {
    glEnable(GL_LIGHTING);
    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < tris.size(); ++i) {
        if ( tris[i].del ) // Ignore deleted facets
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
    glDisable(GL_DEPTH_TEST);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < points.size(); ++i) {
        glColor4f(points[i].col.r, points[i].col.g, points[i].col.b, points[i].col.a);
        glVertex3f(points[i].x,points[i].y,points[i].z);
    }
    glEnd();
    glEnable(GL_DEPTH_TEST);
}

static void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
    glTranslated(0,0,-6);
    //glRotated(xRot,0,1,0);
    //glRotated(yRot,1,0,0);
    glMultMatrixf(rotMat);
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
    return vol > 0.001 ? 1 : vol < -.001 ? -1 : 0;
}

// Distance from point to the plane described by a triangle
// Equation (12,13) of http://mathworld.wolfram.com/Point-PlaneDistance.html
static float distPointToTri( const Point& p, const Tri& t ) {
    Point t1(t.b.x - t.a.x, t.b.y - t.a.y, t.b.z - t.a.z); // b - a
    Point t2(t.c.x - t.a.x, t.c.y - t.a.y, t.c.z - t.a.z); // c - a

    Point n = crossProd(t1, t2);
    float nSize = sqrtf(dotProd(n, n));
    Point nhat(n.x / nSize, n.y / nSize, n.z / nSize);

    Point t3(p.x - t.a.x, p.y - t.a.y, p.z - t.a.y); // p - a
    return (dotProd(nhat, t3));
}

// Normalizes a vector to unit length
static Point normalize(const Point& original) {
    float magnitude = sqrtf(dotProd(original, original));
    if (magnitude == 0)
        return original;
    else
        return Point(original.x / magnitude, original.y / magnitude, original.z / magnitude);
}

//Projects a point onto a plane that passes through the origin defined by the given orthagonal vector
//Adapted from http://stackoverflow.com/a/8944143
static Point projectToPlane(const Point& toProject, const Point& vectorOrthoToPlane) {
    Point planeNormal = normalize(vectorOrthoToPlane);
    float p = dotProd(toProject,planeNormal);
    return Point(toProject.x - (planeNormal.x * p), toProject.y - (planeNormal.y * p), toProject.z - (planeNormal.z * p));
}

static void sleep(int millis) {
#ifdef _WIN32
    Sleep(millis);
#else
    this_thread::sleep_for(chrono::milliseconds(millis));
#endif
}

static void pauseForDisplay() {
    display();
    sleep(SLEEP_TIME);
}

// 3D Quickhull Algorithm
// Based on http://www.cise.ufl.edu/~ungor/courses/fall06/papers/QuickHull.pdf
// Visualization:
// - At every iteration, the current partial hull is varying shades of red.
// - The current face F being processed and optimal outside point are colored yellow (if bg is white, pt is blue instead)
// - The neighbors F' of F (including F) are colored green if they are visible from p
// - New faces are drawn from p to the boundary edges of the visible region, each triangle outlined in yellow, magenta, green as drawn
// - The visible (green) faces disappear
static void quickhull(vector<Point> p, vector<Tri>& t) { // Passes a COPY of the points. Intentional: properties of the points are modified
    t.clear();
    pauseForDisplay();

    // Create simplex of 4 points
    // - here we use the first 4 points entered
    int perm[4][4] = { { 0, 1, 2, 3 }, { 0, 1, 3, 2 }, { 0, 2, 3, 1 }, { 1, 2, 3, 0 } };
    for ( int i = 0; i < 4; ++i ) {
        Tri f(p[perm[i][0]], p[perm[i][1]], p[perm[i][2]]);
        if ( vSign(f, p[perm[i][3]]) < 0 ) { // Make the orientations correct: any other point
                                            // on the hull should be "inside"
            Point temp = f.c;
            f.c = f.b;
            f.b = temp;
        }
        t.push_back(f);
        pauseForDisplay();
    }
    for ( int i = 0; i < 4; ++i ) { // Add adjacent pointers
        for ( int j = 0; j < 4; ++j ) {
            t[i].adjacent.insert(&(t[j]));
        }
    }

    queue<Tri*> t_to_process;
    // for each facet F, for each unassigned point p, if p is above F, assign p to F's outside set
    //      if F has outside points, we will process it later...
    for ( int i = 0; i < 4; ++i ) {
        for ( int j = 0; j < p.size(); ++j ) {
            if ( !p[j].assigned && vSign(t[i], p[j]) < 0 ) {
                t[i].outside_set.insert(&(p[j]));
                p[j].assigned = true;
            }
        }
        if ( t[i].outside_set.size() > 0 ) {
            t_to_process.push(&(t[i]));
        }
    }

    // for each facet F with a non-empty outside set
    while ( t_to_process.size() > 0 ) {
        Tri *f = t_to_process.front();
        t_to_process.pop();

        if ( f->del )
            continue;

        f->col = Color(1,1,0,0.5);
        pauseForDisplay();

        // select the furthest point p of F's outside set
        Point *argmax = 0;
        float max = -1000000;
        //for ( int j = 0; j < f->outside_set.size(); ++j ) {
        for ( set<Point*>::iterator it = f->outside_set.begin(); it != f->outside_set.end(); ++it ) {
            int dist = distPointToTri( *(*it), *f );
            if ( dist > max ) {
                max = dist;
                argmax = *it;
            }
        }

        if ( argmax == 0 )
            continue;

        int argmax_real_index = argmax->index;

        points->at(argmax_real_index).col = bg ? Color(0,0,1,1) : Color(1,1,0,1);
        pauseForDisplay();

        // initialize the visible set V to F
        vector<Tri*> visible;

        // for all neighbors N of facets to F
        for ( set<Tri*>::iterator it = f->adjacent.begin(); it != f->adjacent.end(); ++it ) {
            if ( (*it)->del ) // Ignore deleted facets
                continue;
            // if p is above N, add N to V
            if ( vSign( *(*it), *argmax ) < 0 ) {
                (*it)->vis = true;
                visible.push_back(*it);
                (*it)->col = Color(0,1,0,0.5);
                for ( set<Point*>::iterator it2 = (*it)->outside_set.begin(); it2 != (*it)->outside_set.end(); ++it2 ) {
                    (*it2)->assigned = false;
                }
            }
        }
        pauseForDisplay();

        // the boundary of V is the set of horizon ridges H
        // for each ridge R in H
        for ( int i = 0; i < visible.size(); ++i ) {
            vector<Line> halfEdges = visible[i]->getHalfEdges();
            for ( int j = 0; j < halfEdges.size(); ++j ) {
                Line l = halfEdges[j];
                // SECTION: Determine if the oriented line l is a boundary edge. Else continue
                // l is a boundary edge if it borders exactly one visible face
                /**/bool ridge = true;
                /**/for ( int k = 0; ridge && k < visible.size(); ++k ) {
                /**/    vector<Line> halfEdges = visible[k]->getReverseHalfEdges();
                /**/    for ( int m = 0; m < halfEdges.size(); ++m ) {
                /**/        if ( halfEdges[m] == l ) {
                /**/            ridge = false;
                /**/            break;
                /**/        }
                /**/    }
                /**/}
                /**/if ( !ridge )
                /**/    continue;

                // create a new facet F' from R and p
                Tri newF(l.a,l.b,*argmax);
                lines->clear();
                lines->push_back(Line(newF.a,newF.b,Color(1,1,0,1)));
                lines->push_back(Line(newF.b,newF.c,Color(1,0,1,1)));
                lines->push_back(Line(newF.c,newF.a,Color(0,1,0,1)));
                t.push_back(newF);
                pauseForDisplay();
                // Link the new facet to its neighbors
                // -- two facets are neighbors if they share a vertex
                for ( int k = 0; k < t.size(); ++k ) {
                    if ( t[k].del ) // Ignore deleted facets
                        continue;
                    vector<Point> ptset1 = t[k].getPointSet();
                    vector<Point> ptset2 = t.back().getPointSet();
                    bool done = false;
                    for ( int m = 0; !done && m < ptset1.size(); ++m ) {
                        for ( int n = 0; !done && n < ptset2.size(); ++n ) {
                            if ( ptset1[m] == ptset2[n] ) {
                                t.back().adjacent.insert(&(t[k]));
                                t[k].adjacent.insert(&(t.back()));
                                done = true;
                            }
                        }
                    }
                }

                // For each unassigned point q in an outside set of a facet of V
                for ( set<Tri*>::iterator it = f->adjacent.begin(); it != f->adjacent.end(); ++it ) {
                    if ( !(*it)->vis ) // Ignore invisible facets
                        continue;
                    // if q is above F' add q to F''s outside set
                    for ( set<Point*>::iterator it2 = (*it)->outside_set.begin(); it2 != (*it)->outside_set.end(); ++it2 ) {
                        if ( !(*it2)->assigned && vSign(t.back(), **it2) < 0 ) {
                            t.back().outside_set.insert(*it2);
                            (*it2)->assigned = true;
                        }
                    }
                }

                if ( t.back().outside_set.size() > 0 ) {
                    t_to_process.push(&(t.back()));
                }
                lines->clear();
                pauseForDisplay();
            }
        }
        // Delete the facets in V
        for ( set<Tri*>::iterator it = f->adjacent.begin(); it != f->adjacent.end(); ++it ) {
            (*it)->del = (*it)->vis;
        }
        points->at(argmax_real_index).col = Color(1,0,0,1);
        pauseForDisplay();
    }
}

// 3D Gift Wrapping Algorithm
// Based on Pg 109 of O’Rourke Computational Geometry in C
// Visualization:
// - The algorithm generates an initial face of the convex hull and outlines it in blue.
// - At every iteration, the edge that is being "pivoted around" is highlighted in yellow, as is the third vertex of the "pivoting" triangle.
// - If a new face is generated this iteration, its other two edges are highlighted in green.
static void giftwrap(vector<Point>& p, vector<Tri>& t) {
    t.clear();;
    lines->clear();
    pauseForDisplay();

    //Find the first triangle on the hull (the highest 3 points on the y-axis)
    Point *p1 = &p[0], *p2 = &p[1], *p3 = &p[2];
    for (vector<Point>::iterator it = p.begin(); it != p.end(); ++it) {
        if (it->y > p3->y) {
            if (it->y > p2->y) {
                p3 = p2;
                if (it->y > p1->y) {
                    p2 = p1;
                    p1 = &(*it);
                }
                else
                    p2 = &(*it);
            }
            else
                p3 = &(*it);
        }
    }

    Tri f(*p1, *p2, *p3);
    t.push_back(f);

    //Highlight the triangle's edges
    lines->push_back(Line(*p1,*p2, Color(0,0,1,1)));
    lines->push_back(Line(*p1,*p3, Color(0,0,1,1)));
    lines->push_back(Line(*p2,*p3, Color(0,0,1,1)));

    pauseForDisplay();

    points->push_back(Point(0,0,0,-1,Color(1,1,0,1)));

    lines->at(0).col = Color(1,1,0,1);

    queue<pair<Line, Point> > to_process;
    set<Line> processed;
    set<Tri> generated;

    //Add each of its edges, paired with its remaining points to the queue
    to_process.push(pair<Line, Point>(Line(*p1, *p2),*p3));
    to_process.push(pair<Line, Point>(Line(*p2, *p3),*p1));
    to_process.push(pair<Line, Point>(Line(*p1, *p3),*p2));

    //Loop until out of edge-point pairs
    while (!to_process.empty()) {
        pair<Line, Point> c = to_process.front();
        to_process.pop();

        //Check to ensure we haven't already tried to generate a triangle with this edge
        if (processed.find(c.first) == processed.end()) {
            processed.insert(c.first);

            //Display highlighting stuff
            lines->at(0).a = c.first.a;
            lines->at(0).b = c.first.b;
            points->at(points->size() - 1).x = c.second.x;
            points->at(points->size() - 1).y = c.second.y;
            points->at(points->size() - 1).z = c.second.z;

            //To find the next point, we must first find the edge's vector
            Point plane_vec = Point(c.first.a.x - c.first.b.x, c.first.a.y - c.first.b.y, c.first.a.z - c.first.b.z);
            //We define a plane for which that vector is orthogonal to it, so that we can project other points onto
            //that plane and reason about their angles as though they were 2d points.
            Point origin = projectToPlane(c.first.a, plane_vec); //origin on plane to find angle around
            Point proj1 = projectToPlane(c.second, plane_vec);   //projected version of currently processing point
            Point vec1 = normalize(Point(proj1.x - origin.x, proj1.y - origin.y, 0)); //vector between origin and projected point

            float max_angle = 0;
            Point* best_point = NULL;

            //We then search all points (besides those in the edge-point pair) to find the point with the maximal angle
            //between itself and the point in the edge-point pair around the axis defined by the edge.
            for (vector<Point>::iterator it = p.begin(); it != p.end(); ++it) {
                if (*it != c.first.a && *it != c.first.b && *it != c.second) {
                    Point proj2 = projectToPlane(*it, plane_vec); //projected version of point to check angle
                    Point vec2 = normalize(Point(proj2.x - origin.x, proj2.y - origin.y, 0)); //vector between origin and projected point
                    float angle = acosf(dotProd(vec1, vec2));
                    if (angle > max_angle) {
                        max_angle = angle;
                        best_point = &(*it);
                    }
                }
            }

            if (best_point != NULL) {
                //Generate a triangle using the maximum-angle point and the edge
                Tri hull_tri(c.first.a, c.first.b, *best_point);

                lines->at(1).a = hull_tri.b;
                lines->at(1).b = hull_tri.c;

                lines->at(2).a = hull_tri.c;
                lines->at(2).b = hull_tri.a;

                //Check to ensure this triangle doesn't already exist
                if (generated.find(hull_tri) == generated.end()) {
                    //Add it to the vector of triangles
                    t.push_back(hull_tri);
                    generated.insert(hull_tri);

                    lines->at(1).col = Color(0,1,0,1);
                    lines->at(2).col = Color(0,1,0,1);

                    //Queue its two edges with the appropriate points (we can skip one edge, as we used that to generate this triangle)
                    Line bc(hull_tri.b, hull_tri.c), ca(hull_tri.c, hull_tri.a);
                    to_process.push(pair<Line,Point>(bc, hull_tri.a));
                    to_process.push(pair<Line,Point>(ca, hull_tri.b));

                    pauseForDisplay();
                }
            }
        }
    }

    lines->clear();
    points->pop_back();
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

//Determines scale of rendered scene to fit everything onscreen
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

    scale = DISPLAY_SIZE / furthest;
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
        case 'g':
            giftwrap(*points, *faces);
            break;
        case 'b':
            bg ^= 1;
            glClearColor(bg,bg,bg,1);
            break;
        case 'r':
            lines->clear();
            faces->clear();
            for ( int i = 0; i < points->size(); ++i ) {
                points->at(i).col = Color(1,0,0,1);
            }
            break;
        case 'a':
            //xRot -= rotSpeed;
            glPushMatrix();
            glRotatef(-rotSpeed, 0, 1, 0);
            glMultMatrixf(rotMat);
            glGetFloatv (GL_MODELVIEW_MATRIX, rotMat);
            glPopMatrix();
            break;
        case 'd':
            glPushMatrix();
            glRotatef(rotSpeed, 0, 1, 0);
            glMultMatrixf(rotMat);
            glGetFloatv (GL_MODELVIEW_MATRIX, rotMat);
            glPopMatrix();
            break;
        case 'w':
            glPushMatrix();
            glRotatef(-rotSpeed, 1, 0, 0);
            glMultMatrixf(rotMat);
            glGetFloatv (GL_MODELVIEW_MATRIX, rotMat);
            glPopMatrix();
            break;
        case 's':
            glPushMatrix();
            glRotatef(rotSpeed, 1, 0, 0);
            glMultMatrixf(rotMat);
            glGetFloatv (GL_MODELVIEW_MATRIX, rotMat);
            glPopMatrix();
            break;
    }

    glutPostRedisplay();
}

static void idle(void)
{
    glutPostRedisplay();
}

const GLfloat light_ambient[]  = { 0.2f, 0.2f, 0.2f, 1.0f };
const GLfloat light_diffuse[]  = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_position[] = { 0.0f, 0.0f, 4.5f, 0.0f };

const GLfloat mat_ambient[]    = { 0.7f, 0.7f, 0.7f, 0.5f };
const GLfloat mat_diffuse[]    = { 0.8f, 0.8f, 0.8f, 0.5f };
const GLfloat mat_specular[]   = { 1.0f, 1.0f, 1.0f, 0.5f };
const GLfloat high_shininess[] = { 75.0f };

/* Program entry point */

int main(int argc, char *argv[])
{
    char* file_name;
    bool using_file = false;
    for ( int i = 1; i < argc; ++i ) {
        if ( strcmp(argv[i], "--pointset") == 0 ) {
            if ( i + 1 >= argc ) {
                cerr << "File not specified!";
                return EXIT_FAILURE;
            }
            file_name = argv[i+1];
            using_file = true;
            ++i;
            continue;
        }
        if ( strcmp(argv[i], "--pausetime") == 0 ) {
            if ( i + 1 >= argc ) {
                cerr << "Pause not specified!";
                return EXIT_FAILURE;
            }
            SLEEP_TIME = atoi(argv[i+1]);
            ++i;
            continue;
        }
        if ( strcmp(argv[i], "--pointsize") == 0 ) {
            if ( i + 1 >= argc ) {
                cerr << "Point size not specified!";
                return EXIT_FAILURE;
            }
            POINT_SIZE = atoi(argv[i+1]);
            ++i;
            continue;
        }
        if ( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0 ) {
            cout << "Usage: " << argv[0] << " [OPTIONS]\n"
                << "Options:\n"
                << "\t--pausetime [INT]\t\t\tMilliseconds to pause during visualizations, in ms (default 500)\n"
                << "\t--pointset [STRING]\t\t\tFile to read point set from,\n"
                << "\t\t\t\t\t\tpoints should be specified in \"x y z\" format, one point per line\n"
                << "\t--pointsize [INT]\t\t\tRadius of each point, in pixels (default 2)\n"
                << "In-program commands:\n"
                << "\tw/a/s/d\t\t\t\t\tRotate up/left/down/right\n"
                << "\tb\t\t\t\t\tSwitch the background color between black and white\n"
                << "\tg\t\t\t\t\tPerform 3D Giftwrapping\n"
                << "\tq\t\t\t\t\tPerform 3D QuickHull\n"
                << "\tr\t\t\t\t\tReset all lines/faces (leaving only the pointset)\n"
                << "\t[ESC]\t\t\t\t\tExit the program\n";
            return EXIT_SUCCESS;
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

    Point pmin, pmax;
    for (vector<Point>::iterator it = p.begin(); it != p.end(); ++it) {
        if (it->x < pmin.x)
            pmin.x = it->x;
        if (it->y < pmin.y)
            pmin.y = it->y;
        if (it->z < pmin.z)
            pmin.z = it->z;


        if (it->x > pmax.x)
            pmax.x = it->x;
        if (it->y > pmax.y)
            pmax.y = it->y;
        if (it->z > pmax.z)
            pmax.z = it->z;
    }

    Point pavg((pmax.x - pmin.x)/2, (pmax.y - pmin.y)/2, (pmax.z - pmin.z)/2);

    for (vector<Point>::iterator it = p.begin(); it != p.end(); ++it) {
        it->x -= pavg.x;
        it->y -= pavg.y;
        it->z -= pavg.z;
    }

    vector<Tri> t;
    t.reserve(N * N * N); //

    vector<Line> l;

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

    glClearColor(bg,bg,bg,1);

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
