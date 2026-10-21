/*****************************************************************/
/*    FILE: RegionGeom.cpp                                       */
/*    See RegionGeom.h                                           */
/*****************************************************************/

#include "RegionGeom.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <sstream>

using namespace std;

namespace RegionGeom {

//---------------------------------------------------------------
// Small helpers

static double cross(const Pt& o, const Pt& a, const Pt& b)
{
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

static double distPtSeg(double px, double py, const Pt& a, const Pt& b)
{
  double dx = b.x - a.x, dy = b.y - a.y;
  double len2 = dx * dx + dy * dy;
  double t = 0;
  if(len2 > 0) {
    t = ((px - a.x) * dx + (py - a.y) * dy) / len2;
    if(t < 0) t = 0;
    if(t > 1) t = 1;
  }
  double cx = a.x + t * dx - px, cy = a.y + t * dy - py;
  return sqrt(cx * cx + cy * cy);
}

static bool onSeg(const Pt& a, const Pt& b, const Pt& p)
{
  return min(a.x, b.x) - 1e-9 <= p.x && p.x <= max(a.x, b.x) + 1e-9 &&
         min(a.y, b.y) - 1e-9 <= p.y && p.y <= max(a.y, b.y) + 1e-9;
}

static bool segsTouch(const Pt& a, const Pt& b, const Pt& c, const Pt& d)
{
  double d1 = cross(c, d, a), d2 = cross(c, d, b);
  double d3 = cross(a, b, c), d4 = cross(a, b, d);
  if(((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
     ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
    return true;
  if(fabs(d1) < 1e-12 && onSeg(c, d, a)) return true;
  if(fabs(d2) < 1e-12 && onSeg(c, d, b)) return true;
  if(fabs(d3) < 1e-12 && onSeg(a, b, c)) return true;
  if(fabs(d4) < 1e-12 && onSeg(a, b, d)) return true;
  return false;
}

static bool samePt(const Pt& a, const Pt& b)
{
  return a.x == b.x && a.y == b.y;
}

//---------------------------------------------------------------

double signedArea(const Ring& r)
{
  double a = 0;
  for(size_t i = 0; i < r.size(); i++) {
    const Pt& p = r[i];
    const Pt& q = r[(i + 1) % r.size()];
    a += p.x * q.y - q.x * p.y;
  }
  return a / 2;
}

bool selfIntersects(const Ring& r)
{
  size_t n = r.size();
  if(n < 3)
    return true;
  for(size_t i = 0; i < n; i++) {
    for(size_t j = i + 1; j < n; j++) {
      if(j == i + 1 || (i == 0 && j == n - 1))
        continue;   // neighbouring edges share a vertex
      if(segsTouch(r[i], r[(i + 1) % n], r[j], r[(j + 1) % n]))
        return true;
    }
  }
  return false;
}

bool contains(const Ring& r, double x, double y)
{
  bool in = false;
  for(size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
    if(((r[i].y > y) != (r[j].y > y)) &&
       (x < (r[j].x - r[i].x) * (y - r[i].y) / (r[j].y - r[i].y) + r[i].x))
      in = !in;
  }
  return in;
}

double distToBoundary(const Ring& r, double x, double y)
{
  double best = 1e300;
  for(size_t i = 0; i < r.size(); i++)
    best = min(best, distPtSeg(x, y, r[i], r[(i + 1) % r.size()]));
  return best;
}

Ring cleaned(const Ring& r)
{
  Ring out;
  for(size_t i = 0; i < r.size(); i++)
    if(out.empty() || !samePt(out.back(), r[i]))
      out.push_back(r[i]);
  while(out.size() > 1 && samePt(out.front(), out.back()))
    out.pop_back();
  return out;
}

//---------------------------------------------------------------
// inset(): marching squares on the signed distance to the edges.
// The distance is 1-Lipschitz, so linear interpolation between grid
// nodes is off by at most about one cell; adding a cell to the margin
// keeps every contour point at least `margin` from the edges.

vector<Ring> inset(const Ring& rin, double margin, double cell)
{
  vector<Ring> loops;
  Ring r = cleaned(rin);
  if(r.size() < 3 || cell <= 0)
    return loops;

  double minx = r[0].x, maxx = r[0].x, miny = r[0].y, maxy = r[0].y;
  for(size_t i = 1; i < r.size(); i++) {
    minx = min(minx, r[i].x); maxx = max(maxx, r[i].x);
    miny = min(miny, r[i].y); maxy = max(maxy, r[i].y);
  }
  double pad = 2 * cell;
  double x0 = minx - pad, y0 = miny - pad;
  int nx = (int)ceil((maxx + pad - x0) / cell);
  int ny = (int)ceil((maxy + pad - y0) / cell);
  int W = nx + 1;
  double meff = margin + cell;

  vector<double> v((size_t)W * (ny + 1));
  for(int j = 0; j <= ny; j++) {
    for(int i = 0; i <= nx; i++) {
      double x = x0 + i * cell, y = y0 + j * cell;
      double d = distToBoundary(r, x, y);
      v[(size_t)j * W + i] = (contains(r, x, y) ? d : -d) - meff;
    }
  }
  // Inside the working area <=> v > 0. The padding row/column is outside.
  #define VAL(i, j) v[(size_t)(j) * W + (i)]
  // Edge keys: horizontal (i,j)-(i+1,j) = 2*(j*W+i); vertical (i,j)-(i,j+1) = +1
  map<long long, Pt> pts;
  vector<pair<long long, long long> > segs;
  map<long long, vector<int> > at;

  for(int j = 0; j < ny; j++) {
    for(int i = 0; i < nx; i++) {
      double va = VAL(i, j), vb = VAL(i + 1, j);
      double vc = VAL(i + 1, j + 1), vd = VAL(i, j + 1);
      bool a = va > 0, b = vb > 0, c = vc > 0, d = vd > 0;
      if(a == b && b == c && c == d)
        continue;
      long long kB = 2LL * ((long long)j * W + i);         // bottom a-b
      long long kR = 2LL * ((long long)j * W + i + 1) + 1;  // right  b-c
      long long kT = 2LL * ((long long)(j + 1) * W + i);   // top    d-c
      long long kL = 2LL * ((long long)j * W + i) + 1;      // left   a-d
      double x = x0 + i * cell, y = y0 + j * cell;
      if(a != b) pts[kB] = Pt(x + cell * va / (va - vb), y);
      if(b != c) pts[kR] = Pt(x + cell, y + cell * vb / (vb - vc));
      if(d != c) pts[kT] = Pt(x + cell * vd / (vd - vc), y + cell);
      if(a != d) pts[kL] = Pt(x, y + cell * va / (va - vd));

      vector<long long> k;
      if(a != b) k.push_back(kB);
      if(b != c) k.push_back(kR);
      if(d != c) k.push_back(kT);
      if(a != d) k.push_back(kL);
      vector<pair<long long, long long> > here;
      if(k.size() == 2)
        here.push_back(make_pair(k[0], k[1]));
      else if(k.size() == 4) {
        // Saddle: decide which corners are cut off using the centre.
        bool centre = (va + vb + vc + vd) / 4 > 0;
        bool cut_bd = (a && c) ? centre : !centre;
        if(cut_bd) {
          here.push_back(make_pair(kB, kR));   // corner b
          here.push_back(make_pair(kT, kL));   // corner d
        }
        else {
          here.push_back(make_pair(kL, kB));   // corner a
          here.push_back(make_pair(kR, kT));   // corner c
        }
      }
      for(size_t s = 0; s < here.size(); s++) {
        int id = (int)segs.size();
        segs.push_back(here[s]);
        at[here[s].first].push_back(id);
        at[here[s].second].push_back(id);
      }
    }
  }
  #undef VAL

  // Chain the segments into closed loops.
  vector<bool> used(segs.size(), false);
  for(size_t s0 = 0; s0 < segs.size(); s0++) {
    if(used[s0])
      continue;
    Ring loop;
    long long start = segs[s0].first;
    long long key = segs[s0].second;
    int seg = (int)s0;
    used[s0] = true;
    loop.push_back(pts[start]);
    bool closed = false;
    for(size_t guard = 0; guard <= segs.size(); guard++) {
      loop.push_back(pts[key]);
      if(key == start) { closed = true; break; }
      const vector<int>& nb = at[key];
      int next = -1;
      for(size_t q = 0; q < nb.size(); q++)
        if(nb[q] != seg && !used[nb[q]]) { next = nb[q]; break; }
      if(next < 0)
        break;
      used[next] = true;
      seg = next;
      key = (segs[next].first == key) ? segs[next].second : segs[next].first;
    }
    if(!closed)
      continue;
    loop = cleaned(loop);
    if(loop.size() < 3 || fabs(signedArea(loop)) < cell * cell)
      continue;
    if(signedArea(loop) < 0)
      reverse(loop.begin(), loop.end());
    loops.push_back(loop);
  }

  struct ByArea {
    bool operator()(const Ring& p, const Ring& q) const {
      return fabs(signedArea(p)) > fabs(signedArea(q));
    }
  };
  sort(loops.begin(), loops.end(), ByArea());
  return loops;
}

//---------------------------------------------------------------
// simplify(): Douglas-Peucker on the two halves of the closed ring.

static void dp(const Ring& r, size_t i0, size_t i1, double tol, vector<bool>& keep)
{
  if(i1 <= i0 + 1)
    return;
  double best = -1;
  size_t bi = i0;
  for(size_t i = i0 + 1; i < i1; i++) {
    double d = distPtSeg(r[i].x, r[i].y, r[i0], r[i1 % r.size()]);
    if(d > best) { best = d; bi = i; }
  }
  if(best > tol) {
    keep[bi] = true;
    dp(r, i0, bi, tol, keep);
    dp(r, bi, i1, tol, keep);
  }
}

Ring simplify(const Ring& rin, double tol)
{
  Ring r = cleaned(rin);
  if(r.size() <= 3 || tol <= 0)
    return r;
  // split at vertex 0 and the vertex farthest from it
  size_t far = 0;
  double best = -1;
  for(size_t i = 1; i < r.size(); i++) {
    double dx = r[i].x - r[0].x, dy = r[i].y - r[0].y;
    if(dx * dx + dy * dy > best) { best = dx * dx + dy * dy; far = i; }
  }
  vector<bool> keep(r.size(), false);
  keep[0] = keep[far] = true;
  dp(r, 0, far, tol, keep);
  dp(r, far, r.size(), tol, keep);   // index r.size() wraps to vertex 0
  Ring out;
  for(size_t i = 0; i < r.size(); i++)
    if(keep[i])
      out.push_back(r[i]);
  if(out.size() < 3)
    return r;
  return out;
}

//---------------------------------------------------------------
// convexPieces(): ear clipping, then greedy merging of neighbours.

static bool isConvexRing(const Ring& r)
{
  for(size_t i = 0; i < r.size(); i++)
    if(cross(r[i], r[(i + 1) % r.size()], r[(i + 2) % r.size()]) < -1e-9)
      return false;
  return true;
}

static Ring dropCollinear(const Ring& r)
{
  Ring out;
  for(size_t i = 0; i < r.size(); i++) {
    const Pt& p = r[(i + r.size() - 1) % r.size()];
    const Pt& n = r[(i + 1) % r.size()];
    if(fabs(cross(p, r[i], n)) > 1e-9)
      out.push_back(r[i]);
  }
  return out.size() >= 3 ? out : r;
}

static bool inTriangle(const Pt& p, const Pt& a, const Pt& b, const Pt& c)
{
  return cross(a, b, p) >= 0 && cross(b, c, p) >= 0 && cross(c, a, p) >= 0;
}

bool convexPieces(const Ring& rin, vector<Ring>& pieces)
{
  pieces.clear();
  Ring r = dropCollinear(cleaned(rin));
  if(r.size() < 3)
    return false;
  if(signedArea(r) < 0)
    reverse(r.begin(), r.end());

  // Ear clipping
  vector<size_t> idx;
  for(size_t i = 0; i < r.size(); i++)
    idx.push_back(i);
  while(idx.size() > 3) {
    bool clipped = false;
    size_t n = idx.size();
    for(size_t k = 0; k < n; k++) {
      const Pt& a = r[idx[(k + n - 1) % n]];
      const Pt& b = r[idx[k]];
      const Pt& c = r[idx[(k + 1) % n]];
      double cr = cross(a, b, c);
      if(cr <= 1e-12)
        continue;   // reflex or flat corner
      bool empty = true;
      for(size_t q = 0; q < n && empty; q++) {
        size_t m = idx[q];
        if(m == idx[(k + n - 1) % n] || m == idx[k] || m == idx[(k + 1) % n])
          continue;
        if(samePt(r[m], a) || samePt(r[m], b) || samePt(r[m], c))
          continue;
        if(inTriangle(r[m], a, b, c))
          empty = false;
      }
      if(!empty)
        continue;
      Ring tri;
      tri.push_back(a); tri.push_back(b); tri.push_back(c);
      pieces.push_back(tri);
      idx.erase(idx.begin() + k);
      clipped = true;
      break;
    }
    if(!clipped) {
      // Only flat corners left: drop one and carry on.
      bool dropped = false;
      for(size_t k = 0; k < n; k++) {
        const Pt& a = r[idx[(k + n - 1) % n]];
        const Pt& b = r[idx[k]];
        const Pt& c = r[idx[(k + 1) % n]];
        if(fabs(cross(a, b, c)) <= 1e-12) {
          idx.erase(idx.begin() + k);
          dropped = true;
          break;
        }
      }
      if(!dropped) {
        pieces.clear();
        return false;
      }
    }
  }
  if(idx.size() == 3) {
    Ring tri;
    for(size_t k = 0; k < 3; k++)
      tri.push_back(r[idx[k]]);
    if(fabs(signedArea(tri)) > 1e-9)
      pieces.push_back(tri);
  }

  // Merge neighbours sharing an edge while the union stays convex.
  bool merged = true;
  while(merged) {
    merged = false;
    for(size_t p = 0; p < pieces.size() && !merged; p++) {
      for(size_t q = p + 1; q < pieces.size() && !merged; q++) {
        const Ring& P = pieces[p];
        const Ring& Q = pieces[q];
        for(size_t k = 0; k < P.size() && !merged; k++) {
          const Pt& a = P[k];
          const Pt& b = P[(k + 1) % P.size()];
          for(size_t l = 0; l < Q.size(); l++) {
            if(!samePt(Q[l], b) || !samePt(Q[(l + 1) % Q.size()], a))
              continue;
            Ring U;
            for(size_t s = 0; s < P.size(); s++)       // b ... a around P
              U.push_back(P[(k + 1 + s) % P.size()]);
            for(size_t s = 2; s < Q.size(); s++)       // after a, up to before b
              U.push_back(Q[(l + s) % Q.size()]);
            U = dropCollinear(U);
            if(isConvexRing(U)) {
              pieces[p] = U;
              pieces.erase(pieces.begin() + q);
              merged = true;
            }
            break;
          }
        }
      }
    }
  }
  return !pieces.empty();
}

//---------------------------------------------------------------

static string fmt(double v)
{
  char buf[32];
  snprintf(buf, sizeof(buf), "%.1f", v);
  return buf;
}

static string ptsList(const Ring& r)
{
  string s;
  for(size_t i = 0; i < r.size(); i++) {
    if(i) s += ":";
    s += fmt(r[i].x) + "," + fmt(r[i].y);
  }
  return s;
}

string ringSpec(const Ring& r)
{
  return "pts={" + ptsList(r) + "}";
}

string genPolySpec(const Ring& border, const vector<Ring>& pieces)
{
  string s = "border={" + ptsList(border) + "}";
  for(size_t i = 0; i < pieces.size(); i++)
    s += " # pts={" + ptsList(pieces[i]) + "}";
  return s;
}

//---------------------------------------------------------------

RegionResult buildRegion(const Ring& drawn, double margin,
                         bool boat_known, double boat_x, double boat_y)
{
  RegionResult res;
  Ring r = cleaned(drawn);
  if(r.size() < 3) {
    res.error = "a region needs at least 3 corners";
    return res;
  }
  if(selfIntersects(r)) {
    res.error = "the region's edges cross each other";
    return res;
  }
  if(fabs(signedArea(r)) < 1.0) {
    res.error = "the region has no area";
    return res;
  }
  if(signedArea(r) < 0)
    reverse(r.begin(), r.end());

  Ring core;
  double tol = 0;
  if(margin <= 0) {
    core = r;
    res.areas = 1;
  }
  else {
    double minx = r[0].x, maxx = r[0].x, miny = r[0].y, maxy = r[0].y;
    for(size_t i = 1; i < r.size(); i++) {
      minx = min(minx, r[i].x); maxx = max(maxx, r[i].x);
      miny = min(miny, r[i].y); maxy = max(maxy, r[i].y);
    }
    double extent = max(maxx - minx, maxy - miny);
    double cell = max(max(margin / 10.0, extent / 600.0), 0.5);
    tol = cell / 2;
    // Simplifying can move the line outward by up to tol: shrink by that too.
    vector<Ring> loops = inset(r, margin + tol, cell);
    res.effective_margin = margin + tol + cell;
    if(loops.empty()) {
      ostringstream os;
      os << "no room left inside the " << margin
         << " m margin: the region is narrower than " << 2 * margin << " m";
      res.error = os.str();
      return res;
    }
    // A loop inside another would be a hole; a simple polygon's inset has
    // none, so refuse rather than guess.
    for(size_t i = 0; i < loops.size(); i++)
      for(size_t j = 0; j < loops.size(); j++)
        if(i != j && contains(loops[j], loops[i][0].x, loops[i][0].y)) {
          res.error = "the region shape is too complex to shrink";
          return res;
        }
    size_t pick = 0;   // largest
    if(boat_known)
      for(size_t i = 0; i < loops.size(); i++)
        if(contains(loops[i], boat_x, boat_y)) { pick = i; break; }
    res.areas = (int)loops.size();
    if(loops.size() > 1) {
      ostringstream os;
      os << "narrow parts left " << loops.size() << " separate areas; using "
         << ((boat_known && contains(loops[pick], boat_x, boat_y)) ?
             "the one the boat is in" : "the largest");
      res.note = os.str();
    }
    core = loops[pick];
    Ring simple = simplify(core, tol);
    double t = tol;
    while(selfIntersects(simple) && t > 1e-3) {
      t /= 2;
      simple = simplify(core, t);
    }
    if(selfIntersects(simple))
      simple = core;
    core = simple;
  }

  if(!convexPieces(core, res.pieces)) {
    res.error = "could not split the region into convex pieces";
    return res;
  }
  res.core = core;
  res.spec = genPolySpec(core, res.pieces);
  res.ok = true;
  return res;
}

}  // namespace RegionGeom
