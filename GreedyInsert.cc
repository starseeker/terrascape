#include <iostream>
#include <chrono>
using namespace std;
#include "GreedyInsert.h"

#include "Mask.h"
extern ImportMask *MASK;

void TrackedTriangle::update(Subdivision& s)
{
    GreedySubdivision& gs = (GreedySubdivision&)s;
    gs.scanTriangle(*this);
}





GreedySubdivision::GreedySubdivision(Map *map) : Subdivision()
{
    cerr << "GreedySubdivision constructor starting..." << endl;
    H = map;
    cerr << "Assigned map pointer..." << endl;
    heap = new Heap(128);
    cerr << "Created heap..." << endl;

    int w = H->width;
    int h = H->height;
    cerr << "Map dimensions: " << w << "x" << h << endl;

    is_used.init(w, h);
    cerr << "Initialized is_used array..." << endl;
    int x,y;
    cerr << "Starting loop to initialize is_used values..." << endl;
    for(x=0;x<w;x++)
	for(y=0;y<h;y++)
	    is_used(x,y) = DATA_POINT_UNUSED;
    cerr << "Finished initializing is_used values..." << endl;

    cerr << "About to call initMesh..." << endl;


    initMesh(Vec2(0,0),
	     Vec2(0, h-1),
	     Vec2(w-1, h-1),
	     Vec2(w-1, 0));

    is_used(0, 0)     = DATA_POINT_USED;
    is_used(0, h-1)   = DATA_POINT_USED;
    is_used(w-1, h-1) = DATA_POINT_USED;
    is_used(w-1, 0)   = DATA_POINT_USED;

    count = 4;
}





Triangle *GreedySubdivision::allocFace(Edge *e)
{
    Triangle *t = new TrackedTriangle(e);

    heap->insert(t, -1.0);

    return t;
}




void GreedySubdivision::compute_plane(Plane& plane,
				      Triangle& T,
				      Map& map)
{
    const Vec2& p1 = T.point1();
    const Vec2& p2 = T.point2();
    const Vec2& p3 = T.point3();

    Vec3 v1(p1, map(p1[X], p1[Y]));
    Vec3 v2(p2, map(p2[X], p2[Y]));
    Vec3 v3(p3, map(p3[X], p3[Y]));

    plane.init(v1, v2, v3);
}

///////////////////////////
//
// This is indeed an ugly hack.
// It should be replaced
//
static int vec2_y_compar(const void *a,const void *b)
{
    Vec2 &p1=*(Vec2 *)a,
	 &p2=*(Vec2 *)b;

    return (p1[Y]==p2[Y]) ? 0 : (p1[Y] < p2[Y] ? -1 : 1);
}

static void order_triangle_points(Vec2 *by_y,
				  const Vec2& p1,
				  const Vec2& p2,
				  const Vec2& p3)
{
    by_y[0] = p1;
    by_y[1] = p2;
    by_y[2] = p3;

    qsort(by_y,3,sizeof(Vec2),vec2_y_compar);
}



void GreedySubdivision::scan_triangle_line(Plane& plane,
					   int y,
					   real x1, real x2,
					   Candidate& candidate)
{
    int startx = (int)ceil(MIN(x1,x2));
    int endx   = (int)floor(MAX(x1,x2));

    if( startx > endx ) return;

    // Clamp to map boundaries for safety
    startx = MAX(0, MIN(startx, H->width - 1));
    endx = MAX(0, MIN(endx, H->width - 1));
    if( startx > endx ) return;

    // OPTIMIZATION: For large scan lines, use sampling to reduce work
    int line_width = endx - startx + 1;
    int step = 1;
    if (line_width > 50) {
        step = line_width / 25;  // Sample at most 25 points per line
    }

    real z0 = plane(startx, y);
    real dz = plane.a * step;
    real z, diff;
    
    // Early termination: if we already have a very good candidate, 
    // we can skip scanning if the plane error is too small
    real min_worthwhile_error = candidate.import * 0.1; // Only look for candidates 10x better

    for(int x=startx; x<=endx; x+=step)
    {
	if( !is_used(x,y) )
	{
	    z = H->eval(x,y);
	    diff = fabs(z - z0);
	    
	    // Early termination: skip if error is too small to matter
	    if (diff > min_worthwhile_error) {
		candidate.consider(x, y, MASK->apply(x, y, diff));
	    }
	}

	z0 += dz;
    }
}


void GreedySubdivision::scanTriangle(TrackedTriangle& T)
{
    // Calculate triangle bounding box first to determine strategy
    Vec2 by_y[3];
    order_triangle_points(by_y,T.point1(),T.point2(),T.point3());
    Vec2& v0 = by_y[0];
    Vec2& v1 = by_y[1];
    Vec2& v2 = by_y[2];

    int map_width = H->width;
    int map_height = H->height;
    
    int min_x = (int)floor(MIN(MIN(v0[X], v1[X]), v2[X]));
    int max_x = (int)ceil(MAX(MAX(v0[X], v1[X]), v2[X]));
    int min_y = (int)floor(MIN(MIN(v0[Y], v1[Y]), v2[Y]));
    int max_y = (int)ceil(MAX(MAX(v0[Y], v1[Y]), v2[Y]));
    
    // Clamp to map boundaries
    min_x = MAX(0, min_x);
    max_x = MIN(map_width - 1, max_x);
    min_y = MAX(0, min_y);
    max_y = MIN(map_height - 1, max_y);
    
    // Early exit if triangle is outside map bounds
    if (min_x > max_x || min_y > max_y) {
        if( T.token != NOT_IN_HEAP )
            heap->kill(T.token);
        return;
    }

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;
    int area = width * height;
    
    // RADICAL OPTIMIZATION: For extremely large triangles, defer processing
    if (area > 50000) {
        // Simply put a placeholder candidate at the triangle center with low priority
        // This allows the algorithm to continue and subdivide the triangle later
        int center_x = (min_x + max_x) / 2;
        int center_y = (min_y + max_y) / 2;
        
        if (!is_used(center_x, center_y)) {
            T.setCandidate(center_x, center_y, 50.0);  // Higher priority to ensure processing
            if( T.token == NOT_IN_HEAP )
                heap->insert(&T, 50.0);
            else
                heap->update(&T, 50.0);
        } else {
            // If center is used, remove from heap
            if( T.token != NOT_IN_HEAP )
                heap->kill(T.token);
        }
        return;
    }

    // For smaller triangles, use the optimized scanning
    Plane z_plane;
    compute_plane(z_plane, T, *H);
    Candidate candidate;
    
    if (area > 5000) {
        // For large triangles, use minimal sampling
        int step = MAX(width, height) / 4;  // 4x4 samples max
        step = MAX(10, step);
        
        for (int y = min_y; y <= max_y; y += step) {
            for (int x = min_x; x <= max_x; x += step) {
                if (x < map_width && y < map_height && !is_used(x, y)) {
                    real z = H->eval(x, y);
                    real plane_z = z_plane(x, y);
                    real diff = fabs(z - plane_z);
                    
                    if (diff > 10.0) {
                        candidate.consider(x, y, MASK->apply(x, y, diff));
                    }
                }
            }
        }
    } else {
        // For smaller triangles, use more thorough sampling
        int step = (area > 500) ? 5 : 2;
        
        for (int y = min_y; y <= max_y; y += step) {
            for (int x = min_x; x <= max_x; x += step) {
                if (x < map_width && y < map_height && !is_used(x, y)) {
                    real z = H->eval(x, y);
                    real plane_z = z_plane(x, y);
                    real diff = fabs(z - plane_z);
                    
                    if (diff > 1.0) {
                        candidate.consider(x, y, MASK->apply(x, y, diff));
                    }
                }
            }
        }
    }

    /////////////////////////////////
    //
    // We have now found the appropriate candidate point.
    //
    if( candidate.import < 1e-4 )
    {
	if( T.token != NOT_IN_HEAP )
	    heap->kill(T.token);

#ifdef SAFETY
	T.setCandidate(-69, -69, 0.0);
#endif
    }
    else
    {
	assert( !is_used(candidate.x, candidate.y) );

	T.setCandidate(candidate.x, candidate.y, candidate.import);
	if( T.token == NOT_IN_HEAP )
	    heap->insert(&T, candidate.import);
	else
	    heap->update(&T, candidate.import);
    }
}

Edge *GreedySubdivision::select(int sx, int sy, Triangle *t)
{
    if( is_used(sx, sy) )
    {
	cerr << "   WARNING: Tried to reinsert point: " << sx<<" "<<sy<<endl;
	return NULL;
    }

    is_used(sx,sy) = DATA_POINT_USED;
    count++;
    Vec2 point(sx, sy);
    return insert(point, t);
}


int GreedySubdivision::greedyInsert()
{
    heap_node *node = heap->extract();

    if( !node ) return False;

    TrackedTriangle &T = *(TrackedTriangle *)node->obj;
    int sx, sy;
    T.getCandidate(&sx, &sy);

    select(sx, sy, &T);

    return True;
}

real GreedySubdivision::maxError()
{
    heap_node *node = heap->top();

    if( !node )
	return 0.0;

    return node->import;
}

real GreedySubdivision::rmsError()
{
    real err = 0.0;
    int width = H->width;
    int height = H->height;



    for(int i=0; i<width; i++)
	for(int j=0; j<height; j++)
	{
	    real diff = eval(i, j) - H->eval(i, j);
	    err += diff * diff;
	}

    return sqrt(err / (width * height));
}


real GreedySubdivision::eval(int x,int y) 
{
    Vec2 p(x,y);
    Triangle *T = locate(p)->Lface();

    Plane z_plane;
    compute_plane(z_plane, *T, *H);

    return z_plane(x,y);
}
