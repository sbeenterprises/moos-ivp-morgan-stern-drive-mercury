/*****************************************************************/
/*    FILE: RegionGeom.h                                         */
/*                                                               */
/*  Operating-region geometry, plain C++ (no MOOS), unit tested  */
/*  in tests/region_test.cpp.                                    */
/*                                                               */
/*  The operator draws the region's HALT line. The boat works    */
/*  inside a smaller boundary: the drawn polygon shrunk inward   */
/*  by a margin. BHV_OpRegionV26 is given that working boundary  */
/*  as core_poly with breach_dist = margin, so a breach (helm    */
/*  all-stop) happens on or just inside the drawn line, never    */
/*  beyond it. Any simple polygon is accepted, convex or not;    */
/*  BHV_OpRegionV26 needs it as a border plus convex cover       */
/*  pieces, which buildRegion() also produces.                   */
/*****************************************************************/

#ifndef REGION_GEOM_HEADER
#define REGION_GEOM_HEADER

#include <string>
#include <vector>

namespace RegionGeom {

struct Pt {
  double x, y;
  Pt() : x(0), y(0) {}
  Pt(double xx, double yy) : x(xx), y(yy) {}
};
typedef std::vector<Pt> Ring;   // open ring: last vertex != first

// Area, positive when the vertices run counter-clockwise.
double signedArea(const Ring& r);

// True if any two non-adjacent edges touch or cross.
bool selfIntersects(const Ring& r);

// Even-odd point in polygon.
bool contains(const Ring& r, double x, double y);

// Shortest distance from (x,y) to the ring's edges.
double distToBoundary(const Ring& r, double x, double y);

// Drops repeated vertices and a closing vertex equal to the first.
Ring cleaned(const Ring& r);

// The polygon shrunk inward: every point of every returned piece lies at
// least `margin` from the ring's edges (computed on a grid of `cell`
// metres, then made conservative by the grid error). Narrow parts vanish;
// a shape with a neck narrower than 2 x margin splits into pieces. Pieces
// are counter-clockwise, largest first. Empty if nothing is left.
std::vector<Ring> inset(const Ring& r, double margin, double cell);

// Douglas-Peucker on a closed ring; keeps at least 3 vertices.
Ring simplify(const Ring& r, double tol);

// Convex pieces covering a simple ring exactly (ear clipping, then
// neighbouring triangles merged while the result stays convex).
bool convexPieces(const Ring& r, std::vector<Ring>& pieces);

// "border={x,y:...} # pts={x,y:...} # pts={...}" for BHV_OpRegionV26.
std::string genPolySpec(const Ring& border, const std::vector<Ring>& pieces);

// "pts={x,y:...}" for a plain polygon (viewer / logging).
std::string ringSpec(const Ring& r);

struct RegionResult {
  bool ok;
  std::string error;      // why it was refused (ok == false)
  std::string note;       // e.g. "2 separate areas, using the one the boat is in"
  Ring core;              // working boundary (local x,y)
  std::vector<Ring> pieces;
  std::string spec;       // genPolySpec(core, pieces)
  double effective_margin;
  int areas;              // pieces the inset split into
  RegionResult() : ok(false), effective_margin(0), areas(0) {}
};

// Full pipeline: validate the drawn halt line, shrink it by margin (0 =
// use it as drawn), pick the piece the boat is in (else the largest),
// simplify and split into convex pieces. boat_known=false ignores boat_x/y.
RegionResult buildRegion(const Ring& drawn, double margin,
                         bool boat_known, double boat_x, double boat_y);

}  // namespace RegionGeom

#endif
