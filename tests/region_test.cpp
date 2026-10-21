// Unit tests for lib_regiongeom (RegionGeom): the operating region's
// working boundary. Plain g++, no MOOS. Run via tests/run_tests.sh.

#include "RegionGeom.h"
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <string>

using namespace std;
using namespace RegionGeom;

static int g_checks = 0, g_failed = 0;
#define CHECK(cond, msg) do { g_checks++; if(!(cond)) { g_failed++; \
  cout << "FAIL " << __LINE__ << ": " << msg << endl; } } while(0)

static Ring ring(const double* xy, int n)
{
  Ring r;
  for(int i = 0; i < n; i++)
    r.push_back(Pt(xy[2 * i], xy[2 * i + 1]));
  return r;
}

// Every point of the working boundary (vertices and 1 m steps along its
// edges) must be inside the drawn line and at least `margin` from it.
// Then anything within `margin` of the working area - where the helm
// would call a breach - is still inside or on the drawn line.
static double minClearance(const Ring& drawn, const Ring& core, bool& all_inside)
{
  all_inside = true;
  double best = 1e300;
  for(size_t i = 0; i < core.size(); i++) {
    const Pt& a = core[i];
    const Pt& b = core[(i + 1) % core.size()];
    double len = hypot(b.x - a.x, b.y - a.y);
    int steps = (int)ceil(len) + 1;
    for(int s = 0; s <= steps; s++) {
      double t = (double)s / steps;
      double x = a.x + t * (b.x - a.x), y = a.y + t * (b.y - a.y);
      if(!contains(drawn, x, y)) all_inside = false;
      best = min(best, distToBoundary(drawn, x, y));
    }
  }
  return best;
}

static bool piecesOk(const Ring& core, const vector<Ring>& pieces)
{
  double sum = 0;
  for(size_t i = 0; i < pieces.size(); i++) {
    const Ring& p = pieces[i];
    for(size_t k = 0; k < p.size(); k++) {
      const Pt& a = p[k];
      const Pt& b = p[(k + 1) % p.size()];
      const Pt& c = p[(k + 2) % p.size()];
      if((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) < -1e-6)
        return false;   // not convex
    }
    sum += fabs(signedArea(p));
  }
  return fabs(sum - fabs(signedArea(core))) < 1e-3 * fabs(signedArea(core)) + 1e-6;
}

static void testBasics()
{
  const double sq[] = {0,0, 100,0, 100,100, 0,100};
  Ring s = ring(sq, 4);
  CHECK(fabs(signedArea(s) - 10000) < 1e-9, "square area");
  CHECK(contains(s, 50, 50) && !contains(s, 150, 50), "contains");
  CHECK(fabs(distToBoundary(s, 50, 30) - 30) < 1e-9, "distance to edge");
  CHECK(!selfIntersects(s), "square does not self-intersect");
  const double bow[] = {0,0, 100,100, 100,0, 0,100};
  CHECK(selfIntersects(ring(bow, 4)), "bowtie self-intersects");
  Ring closed = s;
  closed.push_back(Pt(0, 0));
  closed.push_back(Pt(0, 0));
  CHECK(cleaned(closed).size() == 4, "closing and repeated vertices dropped");
}

static void testRectangle()
{
  const double xy[] = {0,0, 400,0, 400,200, 0,200};
  Ring d = ring(xy, 4);
  RegionResult r = buildRegion(d, 30, false, 0, 0);
  CHECK(r.ok, "rectangle accepted: " << r.error);
  bool inside;
  double clear = minClearance(d, r.core, inside);
  CHECK(inside, "rectangle core inside the drawn line");
  CHECK(clear >= 30 - 1e-6, "rectangle core at least 30 m from the line, got " << clear);
  CHECK(clear <= 30 + 2 * r.effective_margin, "rectangle core not far inside, got " << clear);
  CHECK(r.areas == 1 && r.note.empty(), "one area");
  CHECK(piecesOk(r.core, r.pieces), "rectangle pieces convex and cover the core");
  CHECK(r.pieces.size() <= 2, "rectangle needs few pieces, got " << r.pieces.size());
  CHECK(r.spec.find("border={") == 0 && r.spec.find("# pts={") != string::npos,
        "gen poly spec format: " << r.spec.substr(0, 60));
  // clockwise input gives the same result
  Ring cw(d.rbegin(), d.rend());
  RegionResult r2 = buildRegion(cw, 30, false, 0, 0);
  CHECK(r2.ok && fabs(fabs(signedArea(r2.core)) - fabs(signedArea(r.core))) < 1,
        "clockwise drawing gives the same working area");
}

static void testNoMargin()
{
  const double xy[] = {0,0, 300,0, 300,300, 0,300};
  Ring d = ring(xy, 4);
  RegionResult r = buildRegion(d, 0, false, 0, 0);
  CHECK(r.ok && r.core.size() == 4 && fabs(signedArea(r.core) - 90000) < 1e-6,
        "margin 0 uses the drawn line as is");
  CHECK(r.pieces.size() == 1, "a convex region is one piece");
}

static void testLShape()
{
  // L: 400 x 400 with the top-right 250 x 250 quarter removed
  const double xy[] = {0,0, 400,0, 400,150, 150,150, 150,400, 0,400};
  Ring d = ring(xy, 6);
  RegionResult r = buildRegion(d, 30, false, 0, 0);
  CHECK(r.ok, "L-shape accepted: " << r.error);
  bool inside;
  double clear = minClearance(d, r.core, inside);
  CHECK(inside && clear >= 30 - 1e-6, "L-shape core inside, clearance " << clear);
  CHECK(piecesOk(r.core, r.pieces), "L-shape pieces convex and cover the core");
  CHECK(r.pieces.size() >= 2, "L-shape is not convex: needs 2+ pieces");
  // the inner corner of the core sits about 30 m from the reflex corner
  CHECK(!contains(r.core, 160, 160), "core keeps clear of the inner corner");
  CHECK(contains(r.core, 75, 300) && contains(r.core, 300, 75), "both arms usable");
}

static void testWindingChannel()
{
  // A zig-zag channel 150 m wide: centreline 0,0 -> 600,400 -> 1200,0 -> 1800,400
  const double xy[] = {0,-75, 600,325, 1200,-75, 1800,325,
                       1800,475, 1200,75, 600,475, 0,75};
  Ring d = ring(xy, 8);
  RegionResult r = buildRegion(d, 30, false, 0, 0);
  CHECK(r.ok, "winding channel accepted: " << r.error);
  bool inside;
  double clear = minClearance(d, r.core, inside);
  CHECK(inside && clear >= 30 - 1e-6, "channel core inside, clearance " << clear);
  CHECK(piecesOk(r.core, r.pieces), "channel pieces convex and cover the core");
  CHECK(r.areas == 1, "channel stays one area");
}

static void testTooNarrow()
{
  const double xy[] = {0,0, 1000,0, 1000,50, 0,50};   // 50 m wide < 2 x 30
  RegionResult r = buildRegion(ring(xy, 4), 30, false, 0, 0);
  CHECK(!r.ok && r.error.find("narrower") != string::npos,
        "a 50 m channel has no room for a 30 m margin: " << r.error);
}

static void testDumbbell()
{
  // two 300 m squares joined by a 40 m neck
  const double xy[] = {0,0, 300,0, 300,130, 500,130, 500,0, 800,0,
                       800,300, 500,300, 500,170, 300,170, 300,300, 0,300};
  Ring d = ring(xy, 12);
  RegionResult r = buildRegion(d, 30, true, 650, 150);
  CHECK(r.ok, "dumbbell accepted: " << r.error);
  CHECK(r.areas == 2, "neck splits it into 2 areas, got " << r.areas);
  CHECK(contains(r.core, 650, 150), "the area with the boat is used");
  CHECK(r.note.find("boat is in") != string::npos, "note says so: " << r.note);
  RegionResult r2 = buildRegion(d, 30, false, 0, 0);
  CHECK(r2.ok && r2.note.find("largest") != string::npos, "no boat: largest area");
}

static void testRefusals()
{
  const double bow[] = {0,0, 300,300, 300,0, 0,300};
  RegionResult r = buildRegion(ring(bow, 4), 30, false, 0, 0);
  CHECK(!r.ok && r.error.find("cross") != string::npos, "crossing edges refused");
  const double two[] = {0,0, 100,100};
  r = buildRegion(ring(two, 2), 30, false, 0, 0);
  CHECK(!r.ok && r.error.find("3 corners") != string::npos, "2 points refused");
  const double flat[] = {0,0, 100,0, 200,0};
  r = buildRegion(ring(flat, 3), 30, false, 0, 0);
  CHECK(!r.ok, "a flat region is refused");
}

static void testLargeRegionIsQuick()
{
  // 5 km x 5 km with a notch: must stay quick (grid capped at ~600 cells)
  const double xy[] = {0,0, 5000,0, 5000,5000, 2600,5000, 2600,2000,
                       2400,2000, 2400,5000, 0,5000};
  clock_t t0 = clock();
  RegionResult r = buildRegion(ring(xy, 8), 30, false, 0, 0);
  double secs = (double)(clock() - t0) / CLOCKS_PER_SEC;
  CHECK(r.ok, "large region accepted: " << r.error);
  CHECK(secs < 3.0, "large region built in " << secs << " s");
  bool inside;
  double clear = minClearance(ring(xy, 8), r.core, inside);
  CHECK(inside && clear >= 30 - 1e-6, "large region clearance " << clear);
  CHECK(r.core.size() < 300, "large region core simplified, " << r.core.size() << " vertices");
}

int main()
{
  testBasics();
  testRectangle();
  testNoMargin();
  testLShape();
  testWindingChannel();
  testTooNarrow();
  testDumbbell();
  testRefusals();
  testLargeRegionIsQuick();
  cout << g_checks << " checks, " << g_failed << " failed" << endl;
  return g_failed ? 1 : 0;
}
