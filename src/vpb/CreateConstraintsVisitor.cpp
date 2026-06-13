#include <vpb/CreateConstraintsVisitor>
#include <vpb/HeightFieldMapper>
#include <vpb/SpatialUtils>

using namespace vpb;

CreateConstraintsVisitor::CreateConstraintsVisitor(osg::HeightField *heightField, const osg::Matrixd &worldToLocal)
    : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    , _worldToLocal(worldToLocal)
{
    _bbox.set(
        heightField->getOrigin(),
        heightField->getOrigin() + osg::Vec3(
            heightField->getXInterval() * heightField->getNumColumns(),
            heightField->getYInterval() * heightField->getNumRows(),
            0.0f
        )
    );

    _heightFieldMapper = new HeightFieldMapper(*heightField);
    _heightFieldMapper->setMode(HeightFieldMapper::PER_VERTEX);
}

CreateConstraintsVisitor::~CreateConstraintsVisitor()
{
    delete _heightFieldMapper;
    _heightFieldMapper = NULL;
}

void CreateConstraintsVisitor::apply(osg::Geometry &geometry)
{
    if (!_bbox.intersects(geometry.getBoundingBox()))
        return;

    // Clone geometry to avoid HeightFieldMapperArrayVisitor permanenly removing vertices not covered by this particular tile
    //osg::ref_ptr<osg::Geometry> clonedGeometry = dynamic_cast<osg::Geometry *>(geometry.clone(osg::CopyOp::DEEP_COPY_ALL));
    //if (!_heightFieldMapper->map(*clonedGeometry))
    //    return;

    // Always Vec3d and one POLYGON DrawArrays from osgdb_shp
    osg::Vec3dArray *vertices = dynamic_cast<osg::Vec3dArray *>(geometry.getVertexArray());
    if (vertices) {
        osg::Geometry::PrimitiveSetList &prims = geometry.getPrimitiveSetList();
        // Should be only one primitive
        for (osg::Geometry::PrimitiveSetList::iterator it = prims.begin(); it != prims.end(); ++it) {
            osg::DrawArrays *drawArrays = dynamic_cast<osg::DrawArrays *>(it->get());
            if (drawArrays) {
                osg::Vec3d firstCoord;
                unsigned int first = drawArrays->getFirst();
                unsigned int count = drawArrays->getCount();
                unsigned int edgeOffset = _vertices.size();
                unsigned int numEdgesAdded = 0;
                for (unsigned int i = 0; i < count; ++i) {
                    unsigned int index = i + first;
                    if (index < vertices->size()) {
                        osg::Vec3d vertex = (*vertices)[index];
                        double z = _heightFieldMapper->getZfromXY(vertex.x(), vertex.y());
                        if (z == DBL_MAX)
                            continue;
                        vertex._v[2] = z;

                        osg::Vec3d coord = computeLocalPosition(_worldToLocal, vertex);
                        if (i == 0)
                            firstCoord = coord;
                        else if (i < count - 1) {
                            _edges.push_back(CDT::Edge(edgeOffset + numEdgesAdded, edgeOffset + numEdgesAdded + 1));
                            ++numEdgesAdded;
                        }

                        if (i < count - 1 || coord != firstCoord) {
                            _vertices.push_back(CDT::V2d<float>(coord.x(), coord.y()));
                            _heights.push_back(coord.z());
                        }
                    }
                }
                if (numEdgesAdded > 0) {
                    _edges.push_back(CDT::Edge(edgeOffset + numEdgesAdded, edgeOffset));
                }
            }
        }
    }
}

std::vector<CDT::V2d<float> > CreateConstraintsVisitor::getVertices()
{
    return _vertices;
}

CDT::EdgeVec CreateConstraintsVisitor::getEdges()
{
    return _edges;
}

std::vector<float> CreateConstraintsVisitor::getHeights()
{
    return _heights;
}

