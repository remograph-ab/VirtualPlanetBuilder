#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <osg/CoordinateSystemNode>
#include <osg/Math>
#include <osg/PrimitiveSet>

#include <vpb/CreateConstraintsVisitor>
#include <vpb/DataSet>
#include <vpb/Source>
#include <vpb/SourceData>
#include <vpb/SpatialUtils>

using namespace vpb;

namespace
{
    // Orientation sign of the triple (a, b, c) in 2D: >0 counter-clockwise, <0 clockwise,
    // 0 collinear.
    inline double orient2d(double ax, double ay, double bx, double by, double cx, double cy)
    {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    }

    inline bool onSegment(double ax, double ay, double bx, double by, double px, double py)
    {
        return std::min(ax, bx) <= px && px <= std::max(ax, bx) &&
               std::min(ay, by) <= py && py <= std::max(ay, by);
    }

    // True when segment (a0,a1) and segment (b0,b1) intersect (including touching endpoints and
    // collinear overlap).
    inline bool segmentsIntersect(const osg::Vec2d &a0, const osg::Vec2d &a1,
                                  const osg::Vec2d &b0, const osg::Vec2d &b1)
    {
        double d1 = orient2d(b0.x(), b0.y(), b1.x(), b1.y(), a0.x(), a0.y());
        double d2 = orient2d(b0.x(), b0.y(), b1.x(), b1.y(), a1.x(), a1.y());
        double d3 = orient2d(a0.x(), a0.y(), a1.x(), a1.y(), b0.x(), b0.y());
        double d4 = orient2d(a0.x(), a0.y(), a1.x(), a1.y(), b1.x(), b1.y());

        if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
            ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)))
            return true;

        if (d1 == 0.0 && onSegment(b0.x(), b0.y(), b1.x(), b1.y(), a0.x(), a0.y())) return true;
        if (d2 == 0.0 && onSegment(b0.x(), b0.y(), b1.x(), b1.y(), a1.x(), a1.y())) return true;
        if (d3 == 0.0 && onSegment(a0.x(), a0.y(), a1.x(), a1.y(), b0.x(), b0.y())) return true;
        if (d4 == 0.0 && onSegment(a0.x(), a0.y(), a1.x(), a1.y(), b1.x(), b1.y())) return true;

        return false;
    }

    // Squared distance from point (px,py) to the closest point on segment (ax,ay)-(bx,by).
    inline double pointSegmentDistanceSq(double px, double py,
                                         double ax, double ay, double bx, double by)
    {
        double dx = bx - ax;
        double dy = by - ay;
        double lenSq = dx * dx + dy * dy;
        double t = (lenSq > 0.0) ? ((px - ax) * dx + (py - ay) * dy) / lenSq : 0.0;
        if (t < 0.0) t = 0.0;
        else if (t > 1.0) t = 1.0;
        double cx = ax + t * dx;
        double cy = ay + t * dy;
        double ex = px - cx;
        double ey = py - cy;
        return ex * ex + ey * ey;
    }

    // Squared minimum distance between segments (a0,a1) and (b0,b1); 0 when they intersect.
    inline double segmentSegmentDistanceSq(const osg::Vec2d &a0, const osg::Vec2d &a1,
                                           const osg::Vec2d &b0, const osg::Vec2d &b1)
    {
        if (segmentsIntersect(a0, a1, b0, b1)) return 0.0;
        double d = pointSegmentDistanceSq(a0.x(), a0.y(), b0.x(), b0.y(), b1.x(), b1.y());
        d = std::min(d, pointSegmentDistanceSq(a1.x(), a1.y(), b0.x(), b0.y(), b1.x(), b1.y()));
        d = std::min(d, pointSegmentDistanceSq(b0.x(), b0.y(), a0.x(), a0.y(), a1.x(), a1.y()));
        d = std::min(d, pointSegmentDistanceSq(b1.x(), b1.y(), a0.x(), a0.y(), a1.x(), a1.y()));
        return d;
    }

    // Fraction of the polygon edge length used as the near-touch tolerance below: a centerline
    // that comes within this distance of the edge is treated as crossing it. Relative to the
    // edge length so it stays independent of the coordinate system's units.
    const double kEdgeCrossToleranceFraction = 0.05;

    // True when the polygon edge (p0,p1) is crossed by, or almost touches, any road centerline
    // segment. A small tolerance relative to the edge length lets a centerline that merely
    // grazes the edge still count as a cross-section (lateral).
    bool edgeCrossedByLine(const osg::Vec3d &p0, const osg::Vec3d &p1,
                           const std::vector<std::pair<osg::Vec2d, osg::Vec2d> > &segments)
    {
        osg::Vec2d e0(p0.x(), p0.y());
        osg::Vec2d e1(p1.x(), p1.y());

        double ex = e1.x() - e0.x();
        double ey = e1.y() - e0.y();
        double edgeLength = std::sqrt(ex * ex + ey * ey);
        double tolerance = edgeLength * kEdgeCrossToleranceFraction;
        double toleranceSq = tolerance * tolerance;

        for (size_t s = 0; s < segments.size(); ++s)
        {
            if (segmentSegmentDistanceSq(e0, e1, segments[s].first, segments[s].second) <= toleranceSq)
                return true;
        }
        return false;
    }
}

// ---------------------------------------------------------------------------
// ElevationSampler
// ---------------------------------------------------------------------------

ElevationSampler::ElevationSampler(DataSet *dataSet, const osg::CoordinateSystemNode *cs)
    : _dataSet(dataSet)
    , _cs(cs)
    , _verticalScale(dataSet ? dataSet->getVerticalScale() : 1.0)
{
    if (!_dataSet) return;

    // Collect all height field sources, paired with their resolution in the destination
    // coordinate system, so that we can prefer the finest one at each sample point.
    std::vector<std::pair<double, Source *> > sources;
    for (CompositeSource::source_iterator itr(_dataSet->getSourceGraph()); itr.valid(); ++itr)
    {
        Source *source = itr->get();
        if (!source || source->getType() != Source::HEIGHT_FIELD) continue;

        SourceData *sourceData = source->getSourceData();
        if (!sourceData) continue;

        double resolution = sourceData->computeSpatialProperties(_cs).computeResolution();
        sources.push_back(std::pair<double, Source *>(resolution, source));
    }

    // Finest resolution (smallest cell size) first.
    std::sort(sources.begin(), sources.end());

    _elevationSources.reserve(sources.size());
    for (size_t i = 0; i < sources.size(); ++i)
        _elevationSources.push_back(sources[i].second);
}

bool ElevationSampler::sample(double x, double y, float &height) const
{
    for (size_t i = 0; i < _elevationSources.size(); ++i)
    {
        SourceData *sourceData = _elevationSources[i]->getSourceData();
        if (!sourceData) continue;

        float h = 0.0f;
        if (sourceData->sampleElevation(_cs, _verticalScale, x, y, h))
        {
            height = h;
            return true;
        }
    }

    return false;
}

// ---------------------------------------------------------------------------
// CreateConstraintsVisitor
// ---------------------------------------------------------------------------

CreateConstraintsVisitor::CreateConstraintsVisitor(const ElevationSampler *sampler, bool lateral)
    : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    , _sampler(sampler)
    , _lateral(lateral)
{
}

void CreateConstraintsVisitor::apply(osg::Geometry &geometry)
{
    // Always Vec3d and one POLYGON DrawArrays from osgdb_shp.
    osg::Vec3dArray *vertices = dynamic_cast<osg::Vec3dArray *>(geometry.getVertexArray());
    if (!vertices) return;

    osg::Geometry::PrimitiveSetList &prims = geometry.getPrimitiveSetList();
    for (osg::Geometry::PrimitiveSetList::iterator it = prims.begin(); it != prims.end(); ++it)
    {
        osg::DrawArrays *drawArrays = dynamic_cast<osg::DrawArrays *>(it->get());
        if (!drawArrays) continue;

        unsigned int first = drawArrays->getFirst();
        unsigned int count = drawArrays->getCount();

        ConstraintRing ring;
        ring.reserve(count);
        if (_lateral)
        {
            // Road quads: the vertices form a closed polygon (typically a 4-vertex quad per
            // road segment) but their ordering can't be relied upon. Instead, the road
            // centerline (from the line shape file) is used to identify which polygon edges are
            // cross-sections spanning the road width: an edge is a cross-section when the
            // centerline crosses it. Both endpoints of a crossed edge share a single elevation
            // sampled at the edge midpoint, so every cross-section stays laterally flat.
            // Vertices not on any crossed edge fall back to per-vertex sampling. Points outside
            // every source keep a 0.0 height.

            // Gather the unique polygon vertices, dropping the repeated closing vertex.
            ConstraintRing poly;
            poly.reserve(count);
            for (unsigned int i = 0; i < count; ++i)
            {
                unsigned int index = i + first;
                if (index >= vertices->size()) break;
                poly.push_back((*vertices)[index]);
            }
            while (poly.size() >= 2 &&
                   poly.front().x() == poly.back().x() &&
                   poly.front().y() == poly.back().y())
            {
                poly.pop_back();
            }

            size_t n = poly.size();
            std::vector<double> zsum(n, 0.0);
            std::vector<int> zcount(n, 0);

            std::cout.precision(11);

            // Each edge crossed by the centerline is a cross-section: sample its midpoint once
            // and give both endpoints that shared height.
            if (n >= 2)
            {
                for (size_t k = 0; k < n; ++k)
                {
                    size_t a = k;
                    size_t b = (k + 1) % n;
                    if (edgeCrossedByLine(poly[a], poly[b], _lineSegments))
                    {
                        double mx = 0.5 * (poly[a].x() + poly[b].x());
                        double my = 0.5 * (poly[a].y() + poly[b].y());
                        float z = 0.0f;
                        if (_sampler) _sampler->sample(mx, my, z);
                        zsum[a] += z; ++zcount[a];
                        zsum[b] += z; ++zcount[b];
                    }
                }
            }

            for (size_t k = 0; k < n; ++k)
            {
                double z;
                if (zcount[k] > 0)
                {
                    z = zsum[k] / (double)zcount[k];
                }
                else
                {
                    float pz = 0.0f;
                    if (_sampler) _sampler->sample(poly[k].x(), poly[k].y(), pz);
                    z = pz;
                }
                ring.push_back(osg::Vec3d(poly[k].x(), poly[k].y(), z));
            }
        }
        else
        {
            for (unsigned int i = 0; i < count; ++i)
            {
                unsigned int index = i + first;
                if (index >= vertices->size()) break;

                osg::Vec3d vertex = (*vertices)[index];

                // Sample the elevation from all available sources at the highest resolution.
                // Points outside every source keep a 0.0 height, matching the terrain default.
                float z = 0.0f;
                if (_sampler) _sampler->sample(vertex.x(), vertex.y(), z);

                ring.push_back(osg::Vec3d(vertex.x(), vertex.y(), (double)z));
            }
        }

        // Drop the duplicated closing vertex; shape file rings repeat the first point.
        while (ring.size() >= 2 &&
               ring.front().x() == ring.back().x() &&
               ring.front().y() == ring.back().y())
        {
            ring.pop_back();
        }

        if (ring.size() >= 2) _rings.push_back(ring);
    }
}

// ---------------------------------------------------------------------------
// extractLineSegments
// ---------------------------------------------------------------------------

namespace
{
    class ExtractLineSegmentsVisitor : public osg::NodeVisitor
    {
    public:
        explicit ExtractLineSegmentsVisitor(std::vector<std::pair<osg::Vec2d, osg::Vec2d> > &segments)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
            , _segments(segments)
        {
        }

        void apply(osg::Geometry &geometry) override
        {
            osg::Vec3dArray *vertices = dynamic_cast<osg::Vec3dArray *>(geometry.getVertexArray());
            if (!vertices) return;

            osg::Geometry::PrimitiveSetList &prims = geometry.getPrimitiveSetList();
            for (osg::Geometry::PrimitiveSetList::iterator it = prims.begin(); it != prims.end(); ++it)
            {
                osg::DrawArrays *drawArrays = dynamic_cast<osg::DrawArrays *>(it->get());
                if (!drawArrays) continue;

                unsigned int first = drawArrays->getFirst();
                unsigned int count = drawArrays->getCount();
                GLenum mode = drawArrays->getMode();

                if (mode == osg::PrimitiveSet::LINES)
                {
                    for (unsigned int i = 0; i + 1 < count; i += 2)
                    {
                        unsigned int i0 = i + first;
                        unsigned int i1 = i0 + 1;
                        if (i1 >= vertices->size()) break;
                        addSegment((*vertices)[i0], (*vertices)[i1]);
                    }
                }
                else // LINE_STRIP, LINE_LOOP, or anything else with sequential vertices
                {
                    for (unsigned int i = 0; i + 1 < count; ++i)
                    {
                        unsigned int i0 = i + first;
                        unsigned int i1 = i0 + 1;
                        if (i1 >= vertices->size()) break;
                        addSegment((*vertices)[i0], (*vertices)[i1]);
                    }

                    if (mode == osg::PrimitiveSet::LINE_LOOP && count >= 2)
                    {
                        unsigned int i0 = first + count - 1;
                        unsigned int i1 = first;
                        if (i0 < vertices->size() && i1 < vertices->size())
                            addSegment((*vertices)[i0], (*vertices)[i1]);
                    }
                }
            }
        }

    private:
        void addSegment(const osg::Vec3d &a, const osg::Vec3d &b)
        {
            _segments.push_back(std::make_pair(osg::Vec2d(a.x(), a.y()), osg::Vec2d(b.x(), b.y())));
        }

        std::vector<std::pair<osg::Vec2d, osg::Vec2d> > &_segments;
    };
}

void vpb::extractLineSegments(osg::Node &node, std::vector<std::pair<osg::Vec2d, osg::Vec2d> > &segments)
{
    ExtractLineSegmentsVisitor visitor(segments);
    node.accept(visitor);
}

// ---------------------------------------------------------------------------
// buildTileConstraints
// ---------------------------------------------------------------------------

namespace
{

    inline bool insideXY(const osg::Vec3d &p, const vpb::GeospatialExtents &e)
    {
        return p.x() >= e.xMin() && p.x() <= e.xMax() && p.y() >= e.yMin() && p.y() <= e.yMax();
    }

    // Liang-Barsky clipping of the segment a->b against an axis aligned rectangle in XY.
    // Returns false if the segment lies completely outside, otherwise t0/t1 give the
    // parametric entry/exit positions along a->b within [0, 1].
    bool clipSegmentXY(const osg::Vec3d &a, const osg::Vec3d &b, const vpb::GeospatialExtents &e, double &t0, double &t1)
    {
        t0 = 0.0;
        t1 = 1.0;

        double dx = b.x() - a.x();
        double dy = b.y() - a.y();

        double p[4] = {-dx, dx, -dy, dy};
        double q[4] = {a.x() - e.xMin(), e.xMax() - a.x(), a.y() - e.yMin(), e.yMax() - a.y()};

        for (int i = 0; i < 4; ++i)
        {
            if (p[i] == 0.0)
            {
                if (q[i] < 0.0) return false; // parallel to this edge and outside it
            }
            else
            {
                double t = q[i] / p[i];
                if (p[i] < 0.0)
                {
                    if (t > t1) return false;
                    if (t > t0) t0 = t;
                }
                else
                {
                    if (t < t0) return false;
                    if (t < t1) t1 = t;
                }
            }
        }

        return true;
    }

    osg::Vec3 toLocal(const osg::Vec3d &world, const osg::EllipsoidModel *ellipsoid,
                      bool mapLatLongsToXYZ, bool useLocalToTileTransform, const osg::Matrixd &worldToLocal)
    {
        double X = world.x();
        double Y = world.y();
        double Z = world.z();

        if (mapLatLongsToXYZ && ellipsoid)
        {
            ellipsoid->convertLatLongHeightToXYZ(osg::DegreesToRadians(Y), osg::DegreesToRadians(X), Z, X, Y, Z);
        }

        if (useLocalToTileTransform)
            return vpb::computeLocalPosition(worldToLocal, X, Y, Z);

        return osg::Vec3((float)X, (float)Y, (float)Z);
    }

} // unnamed namespace

void vpb::buildTileConstraints(
    const ConstraintRings &rings,
    const GeospatialExtents &tileExtents,
    const osg::EllipsoidModel *ellipsoid,
    bool mapLatLongsToXYZ,
    bool useLocalToTileTransform,
    const osg::Matrixd &worldToLocal,
    std::vector<CDT::V2d<float> > &vertices,
    CDT::EdgeVec &edges,
    std::vector<float> &heights,
    const float& relativeHeight)
{
    for (size_t r = 0; r < rings.size(); ++r)
    {
        const ConstraintRing &ring = rings[r];
        size_t n = ring.size();
        if (n < 2) continue;

        // Fast path: when the whole ring lies inside the tile, emit it directly as a closed
        // loop. This is both quicker and guarantees the closing edge is always present for
        // fully contained constraint areas (the clipping path below only produces the open
        // arcs of rings that actually cross the tile border).
        bool ringFullyInside = true;
        for (size_t i = 0; i < n; ++i)
        {
            if (!insideXY(ring[i], tileExtents)) { ringFullyInside = false; break; }
        }
        if (ringFullyInside)
        {
            unsigned int base = (unsigned int)vertices.size();
            for (size_t i = 0; i < n; ++i)
            {
                osg::Vec3 local = toLocal(ring[i], ellipsoid, mapLatLongsToXYZ, useLocalToTileTransform, worldToLocal);
                vertices.push_back(CDT::V2d<float>(local.x(), local.y()));
                heights.push_back(local.z() + relativeHeight);
            }
            for (unsigned int k = 0; k + 1 < (unsigned int)n; ++k)
                edges.push_back(CDT::Edge(base + k, base + k + 1));
            edges.push_back(CDT::Edge(base + (unsigned int)n - 1, base));
            continue;
        }

        // Clip every ring segment to the tile extents, assembling the surviving pieces into
        // continuous world-space polylines.
        std::vector<std::vector<osg::Vec3d> > polylines;
        std::vector<osg::Vec3d> current;

        for (size_t i = 0; i < n; ++i)
        {
            const osg::Vec3d &a = ring[i];
            const osg::Vec3d &b = ring[(i + 1) % n];

            double t0, t1;
            if (!clipSegmentXY(a, b, tileExtents, t0, t1))
            {
                if (!current.empty())
                {
                    polylines.push_back(current);
                    current.clear();
                }
                continue;
            }

            // Use the exact endpoints when not clipped so continuity comparisons are exact.
            osg::Vec3d ca = (t0 > 0.0) ? (a + (b - a) * t0) : a;
            osg::Vec3d cb = (t1 < 1.0) ? (a + (b - a) * t1) : b;
            bool bClipped = (t1 < 1.0);

            if (current.empty())
            {
                current.push_back(ca);
            }
            else if (current.back() != ca)
            {
                // Discontinuity: the previous segment exited the tile before this one entered.
                polylines.push_back(current);
                current.clear();
                current.push_back(ca);
            }

            if (current.back() != cb) current.push_back(cb);

            if (bClipped)
            {
                polylines.push_back(current);
                current.clear();
            }
        }

        if (!current.empty()) polylines.push_back(current);

        // Join the wrap-around if the first and last polylines meet at the same interior point.
        if (polylines.size() >= 2 && polylines.front().front() == polylines.back().back())
        {
            std::vector<osg::Vec3d> &last = polylines.back();
            std::vector<osg::Vec3d> &firstPolyline = polylines.front();
            last.insert(last.end(), firstPolyline.begin() + 1, firstPolyline.end());
            polylines.erase(polylines.begin());
        }

        // Emit each polyline as constraint vertices, heights and edges.
        for (size_t p = 0; p < polylines.size(); ++p)
        {
            std::vector<osg::Vec3d> &polyline = polylines[p];

            bool closed = polyline.size() >= 2 && polyline.front() == polyline.back();
            if (closed) polyline.pop_back();
            if (polyline.size() < 2) continue;

            unsigned int base = (unsigned int)vertices.size();
            for (size_t k = 0; k < polyline.size(); ++k)
            {
                osg::Vec3 local = toLocal(polyline[k], ellipsoid, mapLatLongsToXYZ, useLocalToTileTransform, worldToLocal);
                vertices.push_back(CDT::V2d<float>(local.x(), local.y()));
                heights.push_back(local.z() + relativeHeight);
            }

            unsigned int cnt = (unsigned int)polyline.size();
            for (unsigned int k = 0; k + 1 < cnt; ++k)
                edges.push_back(CDT::Edge(base + k, base + k + 1));
            if (closed)
                edges.push_back(CDT::Edge(base + cnt - 1, base));
        }
    }
}

void vpb::buildTileConstraintRingsLocal(
    const ConstraintRings &rings,
    const osg::EllipsoidModel *ellipsoid,
    bool mapLatLongsToXYZ,
    bool useLocalToTileTransform,
    const osg::Matrixd &worldToLocal,
    std::vector<std::vector<osg::Vec2> > &localRings)
{
    localRings.clear();
    localRings.reserve(rings.size());

    for (size_t r = 0; r < rings.size(); ++r)
    {
        const ConstraintRing &ring = rings[r];
        if (ring.size() < 3) continue;

        std::vector<osg::Vec2> localRing;
        localRing.reserve(ring.size());
        for (size_t i = 0; i < ring.size(); ++i)
        {
            osg::Vec3 local = toLocal(ring[i], ellipsoid, mapLatLongsToXYZ, useLocalToTileTransform, worldToLocal);
            localRing.push_back(osg::Vec2(local.x(), local.y()));
        }
        localRings.push_back(localRing);
    }
}

// ---------------------------------------------------------------------------
// removeCoveredConstraintEdges
// ---------------------------------------------------------------------------

namespace
{

    // Per-edge geometry on its supporting line, used to detect edge-on coverage.
    struct EdgeGeom
    {
        double ux, uy;        // canonical unit direction (pointing into the upper half plane)
        double off;           // signed perpendicular distance of the line from the origin
        double t0, t1;        // projection of the two endpoints along (ux, uy), t0 <= t1
        double len;           // segment length
        unsigned int idx;     // index into the original edge list
        long angleBin;        // coarse line bucket keys (grouping only, not authoritative)
        long offsetBin;
    };

    // Longest edges first so a covering edge is always processed (and kept) before the
    // shorter edges it covers; ties are broken by original index for stable duplicate removal.
    struct EdgeGeomLonger
    {
        bool operator()(const EdgeGeom &a, const EdgeGeom &b) const
        {
            if (a.len != b.len) return a.len > b.len;
            return a.idx < b.idx;
        }
    };

} // unnamed namespace

void vpb::removeCoveredConstraintEdges(const std::vector<CDT::V2d<float> > &vertices, CDT::EdgeVec &edges)
{
    const size_t numEdges = edges.size();
    if (numEdges < 2) return;

    // Tolerances in the constraint vertices' coordinate space (tile-local units, typically
    // meters). LINE_EPS controls both how close two segments must be to count as collinear
    // and how much slack is allowed when testing coverage. The bin sizes only coarsely group
    // candidate lines; the precise direction/offset tests below decide the actual collinearity.
    const double LINE_EPS = 1e-2;
    const double DIR_DOT = 1.0 - 1e-4;
    const double ANGLE_BIN = 1e-2;
    const double OFFSET_BIN = 1e-1;

    std::vector<EdgeGeom> geoms;
    geoms.reserve(numEdges);
    for (size_t i = 0; i < numEdges; ++i)
    {
        const CDT::V2d<float> &A = vertices[edges[i].v1()];
        const CDT::V2d<float> &B = vertices[edges[i].v2()];
        double dx = (double)B.x - (double)A.x;
        double dy = (double)B.y - (double)A.y;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-9) continue; // degenerate edge: leave it untouched

        double ux = dx / len;
        double uy = dy / len;
        if (uy < 0.0 || (uy == 0.0 && ux < 0.0)) { ux = -ux; uy = -uy; }

        double ta = (double)A.x * ux + (double)A.y * uy;
        double tb = (double)B.x * ux + (double)B.y * uy;

        EdgeGeom g;
        g.ux = ux;
        g.uy = uy;
        g.off = -(double)A.x * uy + (double)A.y * ux;
        g.t0 = std::min(ta, tb);
        g.t1 = std::max(ta, tb);
        g.len = len;
        g.idx = (unsigned int)i;
        g.angleBin = (long)std::floor(std::atan2(uy, ux) / ANGLE_BIN + 0.5);
        g.offsetBin = (long)std::floor(g.off / OFFSET_BIN + 0.5);
        geoms.push_back(g);
    }

    std::sort(geoms.begin(), geoms.end(), EdgeGeomLonger());

    // Bucket of kept edges keyed by their coarse line bin, holding indices into geoms.
    std::map<std::pair<long, long>, std::vector<size_t> > buckets;
    std::vector<bool> removed(numEdges, false);

    for (size_t gi = 0; gi < geoms.size(); ++gi)
    {
        const EdgeGeom &C = geoms[gi];

        bool covered = false;
        for (long da = -1; da <= 1 && !covered; ++da)
        {
            for (long db = -1; db <= 1 && !covered; ++db)
            {
                std::map<std::pair<long, long>, std::vector<size_t> >::iterator it =
                    buckets.find(std::make_pair(C.angleBin + da, C.offsetBin + db));
                if (it == buckets.end()) continue;

                const std::vector<size_t> &kept = it->second;
                for (size_t ki = 0; ki < kept.size(); ++ki)
                {
                    const EdgeGeom &K = geoms[kept[ki]];

                    // Same supporting line? (same direction and same perpendicular offset)
                    if (C.ux * K.ux + C.uy * K.uy < DIR_DOT) continue;
                    if (std::fabs(C.off - K.off) > LINE_EPS) continue;

                    // C completely covered edge-on by K?
                    if (K.t0 <= C.t0 + LINE_EPS && K.t1 >= C.t1 - LINE_EPS)
                    {
                        covered = true;
                        break;
                    }
                }
            }
        }

        if (covered)
        {
            removed[C.idx] = true;
        }
        else
        {
            buckets[std::make_pair(C.angleBin, C.offsetBin)].push_back(gi);
        }
    }

    CDT::EdgeVec filtered;
    filtered.reserve(numEdges);
    for (size_t i = 0; i < numEdges; ++i)
    {
        if (!removed[i]) filtered.push_back(edges[i]);
    }
    edges.swap(filtered);
}

// ---------------------------------------------------------------------------
// separateTouchingConstraintVertices
// ---------------------------------------------------------------------------

namespace
{

    struct SegGeom
    {
        double ax, ay;     // first endpoint
        double bx, by;     // second endpoint
        double dx, dy;     // b - a
        double len2;       // |d|^2
        double len;
    };

} // unnamed namespace

void vpb::separateTouchingConstraintVertices(std::vector<CDT::V2d<float> > &vertices, const CDT::EdgeVec &edges)
{
    const size_t numEdges = edges.size();
    const size_t numVerts = vertices.size();
    if (numEdges < 2 || numVerts < 3) return;

    // Precompute segment geometry, vertex usage and per-vertex incident edges.
    std::vector<SegGeom> segs(numEdges);
    std::vector<unsigned char> referenced(numVerts, 0);
    std::vector<std::vector<unsigned int> > incident(numVerts);

    double minX = DBL_MAX, minY = DBL_MAX, maxX = -DBL_MAX, maxY = -DBL_MAX;
    double sumLen = 0.0;
    size_t validSegs = 0;
    for (size_t i = 0; i < numEdges; ++i)
    {
        unsigned int i1 = edges[i].v1();
        unsigned int i2 = edges[i].v2();
        const CDT::V2d<float> &A = vertices[i1];
        const CDT::V2d<float> &B = vertices[i2];

        SegGeom &s = segs[i];
        s.ax = A.x; s.ay = A.y;
        s.bx = B.x; s.by = B.y;
        s.dx = s.bx - s.ax; s.dy = s.by - s.ay;
        s.len2 = s.dx * s.dx + s.dy * s.dy;
        s.len = std::sqrt(s.len2);

        referenced[i1] = referenced[i2] = 1;
        incident[i1].push_back((unsigned int)i);
        incident[i2].push_back((unsigned int)i);

        minX = std::min(minX, std::min(s.ax, s.bx));
        minY = std::min(minY, std::min(s.ay, s.by));
        maxX = std::max(maxX, std::max(s.ax, s.bx));
        maxY = std::max(maxY, std::max(s.ay, s.by));
        if (s.len > 0.0) { sumLen += s.len; ++validSegs; }
    }
    if (validSegs == 0) return;

    double spanX = maxX - minX;
    double spanY = maxY - minY;
    if (spanX <= 0.0 && spanY <= 0.0) return;

    // Uniform grid sized to the average edge length, capped so the index stays small.
    double cell = sumLen / (double)validSegs;
    if (cell <= 0.0) cell = 1.0;

    const long long maxCells = 1LL << 20;
    long gw = (long)std::floor(spanX / cell) + 1;
    long gh = (long)std::floor(spanY / cell) + 1;
    if (gw < 1) gw = 1;
    if (gh < 1) gh = 1;
    while ((long long)gw * (long long)gh > maxCells)
    {
        cell *= 2.0;
        gw = (long)std::floor(spanX / cell) + 1;
        gh = (long)std::floor(spanY / cell) + 1;
        if (gw < 1) gw = 1;
        if (gh < 1) gh = 1;
    }

    std::vector<std::vector<unsigned int> > grid((size_t)((long long)gw * gh));
    for (size_t i = 0; i < numEdges; ++i)
    {
        const SegGeom &s = segs[i];
        if (s.len2 <= 0.0) continue;

        long cx0 = (long)std::floor((std::min(s.ax, s.bx) - minX) / cell);
        long cx1 = (long)std::floor((std::max(s.ax, s.bx) - minX) / cell);
        long cy0 = (long)std::floor((std::min(s.ay, s.by) - minY) / cell);
        long cy1 = (long)std::floor((std::max(s.ay, s.by) - minY) / cell);
        if (cx0 < 0) cx0 = 0;
        if (cy0 < 0) cy0 = 0;
        if (cx1 >= gw) cx1 = gw - 1;
        if (cy1 >= gh) cy1 = gh - 1;

        for (long cy = cy0; cy <= cy1; ++cy)
            for (long cx = cx0; cx <= cx1; ++cx)
                grid[(size_t)(cy * gw + cx)].push_back((unsigned int)i);
    }

    // For each constraint vertex, find the nearest non-incident edge it lies on (a T-junction)
    // and push the vertex a tiny step off that edge, toward the side its own edges occupy, so
    // the constrained Delaunay triangulation no longer sees touching/intersecting constraints.
    for (unsigned int vi = 0; vi < numVerts; ++vi)
    {
        if (!referenced[vi]) continue;

        double px = vertices[vi].x;
        double py = vertices[vi].y;

        double ulp = std::max(1.0, std::max(std::fabs(px), std::fabs(py))) * (double)FLT_EPSILON;

        long cx = (long)std::floor((px - minX) / cell);
        long cy = (long)std::floor((py - minY) / cell);
        if (cx < 0) cx = 0;
        if (cx >= gw) cx = gw - 1;
        if (cy < 0) cy = 0;
        if (cy >= gh) cy = gh - 1;

        double bestDist = DBL_MAX;
        int bestEdge = -1;
        for (long ny = cy - 1; ny <= cy + 1; ++ny)
        {
            if (ny < 0 || ny >= gh) continue;
            for (long nx = cx - 1; nx <= cx + 1; ++nx)
            {
                if (nx < 0 || nx >= gw) continue;

                const std::vector<unsigned int> &cellEdges = grid[(size_t)(ny * gw + nx)];
                for (size_t ce = 0; ce < cellEdges.size(); ++ce)
                {
                    unsigned int ei = cellEdges[ce];
                    if (edges[ei].v1() == vi || edges[ei].v2() == vi) continue; // shared vertex, not a touch

                    const SegGeom &s = segs[ei];
                    if (s.len2 <= 0.0) continue;

                    double touchTol = std::max(s.len * 1.0e-6, ulp * 4.0);
                    double t = ((px - s.ax) * s.dx + (py - s.ay) * s.dy) / s.len2;

                    // Exclude touches at (or beyond) the from/to endpoints of the other edge.
                    double endTolT = touchTol / s.len;
                    if (t <= endTolT || t >= 1.0 - endTolT) continue;

                    double projx = s.ax + t * s.dx;
                    double projy = s.ay + t * s.dy;
                    double ddx = px - projx;
                    double ddy = py - projy;
                    double dist = std::sqrt(ddx * ddx + ddy * ddy);
                    if (dist > touchTol) continue;

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestEdge = (int)ei;
                    }
                }
            }
        }

        if (bestEdge < 0) continue;

        const SegGeom &F = segs[bestEdge];

        // Unit normal of the touched edge.
        double nx = -F.dy / F.len;
        double ny = F.dx / F.len;

        // Determine which side of F this vertex's own edges occupy. If they straddle F the
        // junction is a genuine crossing rather than a touch, so leave it alone to avoid
        // turning a touch into a real intersection.
        double sideTol = std::max(F.len * 1.0e-9, ulp);
        double netSign = 0.0;
        bool conflict = false;
        const std::vector<unsigned int> &inc = incident[vi];
        for (size_t k = 0; k < inc.size() && !conflict; ++k)
        {
            unsigned int ei = inc[k];
            unsigned int other = (edges[ei].v1() == vi) ? edges[ei].v2() : edges[ei].v1();
            double qx = (double)vertices[other].x - px;
            double qy = (double)vertices[other].y - py;
            double sproj = qx * nx + qy * ny;
            if (sproj > sideTol)
            {
                if (netSign < 0.0) conflict = true;
                else netSign = 1.0;
            }
            else if (sproj < -sideTol)
            {
                if (netSign > 0.0) conflict = true;
                else netSign = -1.0;
            }
        }
        if (conflict || netSign == 0.0) continue;

        double nudge = std::max(F.len * 1.0e-4, ulp * 16.0);
        vertices[vi].x = (float)(px + nx * netSign * nudge);
        vertices[vi].y = (float)(py + ny * netSign * nudge);
    }
}

// ---------------------------------------------------------------------------
// mergeDuplicateConstraintVertices
// ---------------------------------------------------------------------------

namespace
{

    // Exact (bit-equal) key for a constraint vertex position. Coincident vertices coming from
    // shared boundaries of different shapes are produced through identical transforms and so
    // are bit-identical, which makes exact hashing both correct and O(1) per lookup.
    struct VertexKey
    {
        float x, y;
        bool operator==(const VertexKey &o) const { return x == o.x && y == o.y; }
    };

    struct VertexKeyHash
    {
        std::size_t operator()(const VertexKey &k) const
        {
            unsigned int ix, iy;
            std::memcpy(&ix, &k.x, sizeof(ix));
            std::memcpy(&iy, &k.y, sizeof(iy));
            std::size_t h = (std::size_t)ix * 73856093u;
            h ^= (std::size_t)iy * 19349663u + (h << 6) + (h >> 2);
            return h;
        }
    };

} // unnamed namespace

void vpb::mergeDuplicateConstraintVertices(
    std::vector<CDT::V2d<float> > &vertices,
    CDT::EdgeVec &edges,
    std::vector<float> &heights)
{
    const size_t numVerts = vertices.size();
    if (numVerts < 2) return;

    std::vector<unsigned int> remap(numVerts);
    std::vector<CDT::V2d<float> > newVertices;
    std::vector<float> newHeights;
    newVertices.reserve(numVerts);
    newHeights.reserve(numVerts);

    std::unordered_map<VertexKey, unsigned int, VertexKeyHash> lookup;
    lookup.reserve(numVerts * 2);

    for (unsigned int i = 0; i < numVerts; ++i)
    {
        VertexKey key;
        key.x = vertices[i].x;
        key.y = vertices[i].y;
        if (key.x == 0.0f) key.x = 0.0f; // normalize -0.0 to +0.0
        if (key.y == 0.0f) key.y = 0.0f;

        std::unordered_map<VertexKey, unsigned int, VertexKeyHash>::iterator it = lookup.find(key);
        if (it != lookup.end())
        {
            remap[i] = it->second;
        }
        else
        {
            unsigned int ni = (unsigned int)newVertices.size();
            lookup.insert(std::make_pair(key, ni));
            newVertices.push_back(vertices[i]);
            newHeights.push_back(i < heights.size() ? heights[i] : 0.0f);
            remap[i] = ni;
        }
    }

    if (newVertices.size() == numVerts) return; // nothing was merged

    // Remap the edges onto the compacted vertex set, dropping edges that collapsed to a point
    // and exact duplicate edges that arise where different shapes shared a boundary.
    CDT::EdgeVec newEdges;
    newEdges.reserve(edges.size());
    std::unordered_set<unsigned long long> seenEdges;
    seenEdges.reserve(edges.size() * 2);

    for (size_t i = 0; i < edges.size(); ++i)
    {
        unsigned int a = remap[edges[i].v1()];
        unsigned int b = remap[edges[i].v2()];
        if (a == b) continue;

        unsigned int lo = std::min(a, b);
        unsigned int hi = std::max(a, b);
        unsigned long long ekey = ((unsigned long long)lo << 32) | (unsigned long long)hi;
        if (!seenEdges.insert(ekey).second) continue;

        newEdges.push_back(CDT::Edge(a, b));
    }

    vertices.swap(newVertices);
    heights.swap(newHeights);
    edges.swap(newEdges);
}

// ---------------------------------------------------------------------------
// splitIntersectingConstraintEdges
// ---------------------------------------------------------------------------

void vpb::splitIntersectingConstraintEdges(
    std::vector<CDT::V2d<float> > &vertices,
    CDT::EdgeVec &edges,
    std::vector<float> &heights)
{
    const size_t numEdges = edges.size();
    if (numEdges < 2) return;

    // Segment geometry and overall bounding box.
    struct Seg { double ax, ay, bx, by, dx, dy; };
    std::vector<Seg> segs(numEdges);

    double minX = DBL_MAX, minY = DBL_MAX, maxX = -DBL_MAX, maxY = -DBL_MAX;
    double sumLen = 0.0;
    size_t validSegs = 0;
    for (size_t i = 0; i < numEdges; ++i)
    {
        const CDT::V2d<float> &A = vertices[edges[i].v1()];
        const CDT::V2d<float> &B = vertices[edges[i].v2()];
        Seg &s = segs[i];
        s.ax = A.x; s.ay = A.y;
        s.bx = B.x; s.by = B.y;
        s.dx = s.bx - s.ax; s.dy = s.by - s.ay;

        minX = std::min(minX, std::min(s.ax, s.bx));
        minY = std::min(minY, std::min(s.ay, s.by));
        maxX = std::max(maxX, std::max(s.ax, s.bx));
        maxY = std::max(maxY, std::max(s.ay, s.by));
        double len = std::sqrt(s.dx * s.dx + s.dy * s.dy);
        if (len > 0.0) { sumLen += len; ++validSegs; }
    }
    if (validSegs == 0) return;

    double spanX = maxX - minX;
    double spanY = maxY - minY;
    if (spanX <= 0.0 && spanY <= 0.0) return;

    // Uniform grid sized to the average edge length, capped so the index stays small.
    double cell = sumLen / (double)validSegs;
    if (cell <= 0.0) cell = 1.0;

    const long long maxCells = 1LL << 20;
    long gw = (long)std::floor(spanX / cell) + 1;
    long gh = (long)std::floor(spanY / cell) + 1;
    if (gw < 1) gw = 1;
    if (gh < 1) gh = 1;
    while ((long long)gw * (long long)gh > maxCells)
    {
        cell *= 2.0;
        gw = (long)std::floor(spanX / cell) + 1;
        gh = (long)std::floor(spanY / cell) + 1;
        if (gw < 1) gw = 1;
        if (gh < 1) gh = 1;
    }

    std::vector<std::vector<unsigned int> > grid((size_t)((long long)gw * gh));
    for (size_t i = 0; i < numEdges; ++i)
    {
        const Seg &s = segs[i];
        long cx0 = (long)std::floor((std::min(s.ax, s.bx) - minX) / cell);
        long cx1 = (long)std::floor((std::max(s.ax, s.bx) - minX) / cell);
        long cy0 = (long)std::floor((std::min(s.ay, s.by) - minY) / cell);
        long cy1 = (long)std::floor((std::max(s.ay, s.by) - minY) / cell);
        if (cx0 < 0) cx0 = 0;
        if (cy0 < 0) cy0 = 0;
        if (cx1 >= gw) cx1 = gw - 1;
        if (cy1 >= gh) cy1 = gh - 1;
        for (long cy = cy0; cy <= cy1; ++cy)
            for (long cx = cx0; cx <= cx1; ++cx)
                grid[(size_t)(cy * gw + cx)].push_back((unsigned int)i);
    }

    // Intersection points are appended to the vertex/height arrays, deduplicated by exact
    // position (so several edges meeting at one point share a single vertex). Seed the lookup
    // with the existing vertices so a crossing that lands on a vertex reuses it.
    std::unordered_map<VertexKey, unsigned int, VertexKeyHash> pointLookup;
    pointLookup.reserve(vertices.size() * 2);
    for (unsigned int i = 0; i < (unsigned int)vertices.size(); ++i)
    {
        VertexKey key;
        key.x = vertices[i].x == 0.0f ? 0.0f : vertices[i].x;
        key.y = vertices[i].y == 0.0f ? 0.0f : vertices[i].y;
        pointLookup.insert(std::make_pair(key, i)); // first occurrence wins
    }

    // Per-edge list of interior split points as (parameter along the edge, vertex index).
    std::vector<std::vector<std::pair<double, unsigned int> > > splits(numEdges);
    std::unordered_set<unsigned long long> testedPairs;

    for (size_t gc = 0; gc < grid.size(); ++gc)
    {
        const std::vector<unsigned int> &cellEdges = grid[gc];
        for (size_t a = 0; a + 1 < cellEdges.size(); ++a)
        {
            for (size_t b = a + 1; b < cellEdges.size(); ++b)
            {
                unsigned int i = cellEdges[a];
                unsigned int j = cellEdges[b];
                unsigned int lo = std::min(i, j);
                unsigned int hi = std::max(i, j);
                unsigned long long pkey = ((unsigned long long)lo << 32) | (unsigned long long)hi;
                if (!testedPairs.insert(pkey).second) continue; // already handled in another cell

                // Skip edges that share a vertex (adjacent ring edges are not intersections).
                if (edges[i].v1() == edges[j].v1() || edges[i].v1() == edges[j].v2() ||
                    edges[i].v2() == edges[j].v1() || edges[i].v2() == edges[j].v2())
                    continue;

                const Seg &si = segs[i];
                const Seg &sj = segs[j];

                double denom = si.dx * sj.dy - si.dy * sj.dx;
                if (denom == 0.0) continue; // parallel or collinear: handled elsewhere

                double wx = sj.ax - si.ax;
                double wy = sj.ay - si.ay;
                double t = (wx * sj.dy - wy * sj.dx) / denom; // along edge i
                double u = (wx * si.dy - wy * si.dx) / denom; // along edge j

                // Only split where the segments cross strictly in their interiors; endpoint
                // touches (T-junctions, shared corners) are left for the other passes.
                const double EPS = 1e-7;
                if (t <= EPS || t >= 1.0 - EPS || u <= EPS || u >= 1.0 - EPS) continue;

                double px = si.ax + t * si.dx;
                double py = si.ay + t * si.dy;

                VertexKey key;
                key.x = (float)px; if (key.x == 0.0f) key.x = 0.0f;
                key.y = (float)py; if (key.y == 0.0f) key.y = 0.0f;

                unsigned int pidx;
                std::unordered_map<VertexKey, unsigned int, VertexKeyHash>::iterator pit = pointLookup.find(key);
                if (pit != pointLookup.end())
                {
                    pidx = pit->second;
                }
                else
                {
                    float hi0 = edges[i].v1() < heights.size() ? heights[edges[i].v1()] : 0.0f;
                    float hi1 = edges[i].v2() < heights.size() ? heights[edges[i].v2()] : 0.0f;
                    float hj0 = edges[j].v1() < heights.size() ? heights[edges[j].v1()] : 0.0f;
                    float hj1 = edges[j].v2() < heights.size() ? heights[edges[j].v2()] : 0.0f;
                    float hiAt = (float)(hi0 * (1.0 - t) + hi1 * t);
                    float hjAt = (float)(hj0 * (1.0 - u) + hj1 * u);

                    pidx = (unsigned int)vertices.size();
                    vertices.push_back(CDT::V2d<float>(key.x, key.y));
                    heights.push_back(0.5f * (hiAt + hjAt));
                    pointLookup.insert(std::make_pair(key, pidx));
                }

                if (pidx != edges[i].v1() && pidx != edges[i].v2())
                    splits[i].push_back(std::make_pair(t, pidx));
                if (pidx != edges[j].v1() && pidx != edges[j].v2())
                    splits[j].push_back(std::make_pair(u, pidx));
            }
        }
    }

    // Rebuild the edge list, splitting each intersected edge into a chain through its
    // intersection vertices (sorted along the edge). Drop degenerate and duplicate edges.
    CDT::EdgeVec newEdges;
    newEdges.reserve(numEdges);
    std::unordered_set<unsigned long long> seenEdges;
    seenEdges.reserve(numEdges * 2);

    for (size_t i = 0; i < numEdges; ++i)
    {
        if (splits[i].empty())
        {
            unsigned int a = edges[i].v1();
            unsigned int b = edges[i].v2();
            unsigned int lo = std::min(a, b);
            unsigned int hi = std::max(a, b);
            unsigned long long ekey = ((unsigned long long)lo << 32) | (unsigned long long)hi;
            if (seenEdges.insert(ekey).second) newEdges.push_back(edges[i]);
            continue;
        }

        std::sort(splits[i].begin(), splits[i].end());

        unsigned int prev = edges[i].v1();
        for (size_t k = 0; k < splits[i].size(); ++k)
        {
            unsigned int cur = splits[i][k].second;
            if (cur == prev) continue;
            unsigned int lo = std::min(prev, cur);
            unsigned int hi = std::max(prev, cur);
            unsigned long long ekey = ((unsigned long long)lo << 32) | (unsigned long long)hi;
            if (seenEdges.insert(ekey).second) newEdges.push_back(CDT::Edge(prev, cur));
            prev = cur;
        }

        unsigned int last = edges[i].v2();
        if (last != prev)
        {
            unsigned int lo = std::min(prev, last);
            unsigned int hi = std::max(prev, last);
            unsigned long long ekey = ((unsigned long long)lo << 32) | (unsigned long long)hi;
            if (seenEdges.insert(ekey).second) newEdges.push_back(CDT::Edge(prev, last));
        }
    }

    edges.swap(newEdges);
}

