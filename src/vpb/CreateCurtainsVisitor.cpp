#include <algorithm>

#include <vpb/CreateCurtainsVisitor>

#include <osg/BoundingBox>
#include <osg/TriangleIndexFunctor>
#include <osg/PrimitiveSet>

using namespace vpb;

#define TOLERANCE 0.001f
#define CURTAIN_RATIO 0.03f

struct FindEdgesFunctor
{
  FindEdgesFunctor()
  : vertices(NULL)
  , normals(NULL)
  , texCoords(NULL)
  , primitives(new osg::DrawElementsUShort(osg::PrimitiveSet::TRIANGLES))
  , curtainHeight(1.0f)
  {
  }

  void setBbox(const osg::BoundingBox &bb) {
    bbox = bb;
    curtainHeight = CURTAIN_RATIO * std::max<float>(bbox.xMax() - bbox.xMin(), bbox.yMax() - bbox.yMin());
  }

  void setCurtainHeight(const float &ch) {
    curtainHeight = ch;
  }

  void addCurtainIfOnBorder(unsigned int p1, unsigned int p2, const osg::Vec3 &v1, const osg::Vec3 &v2) {
    if (
      (fabsf(v1.x() - bbox.xMin()) < TOLERANCE && fabsf(v2.x() - bbox.xMin()) < TOLERANCE) ||
      (fabsf(v1.x() - bbox.xMax()) < TOLERANCE && fabsf(v2.x() - bbox.xMax()) < TOLERANCE) ||
      (fabsf(v1.y() - bbox.yMax()) < TOLERANCE && fabsf(v2.y() - bbox.yMax()) < TOLERANCE) ||
      (fabsf(v1.y() - bbox.yMin()) < TOLERANCE && fabsf(v2.y() - bbox.yMin()) < TOLERANCE)
    ) {
      primitives->addElement(p2);
      primitives->addElement(p1);

      unsigned int lower1 = vertices->size();
      primitives->addElement(lower1);
      vertices->push_back(osg::Vec3(v1.x(), v1.y(), v1.z() - curtainHeight));
      normals->push_back((*normals)[p1]);
      texCoords->push_back((*texCoords)[p1]);

      primitives->addElement(p2);
      primitives->addElement(lower1);

      unsigned int lower2 = vertices->size();
      primitives->addElement(lower2);
      normals->push_back((*normals)[p2]);
      texCoords->push_back((*texCoords)[p2]);
      vertices->push_back(osg::Vec3(v2.x(), v2.y(), v2.z() - curtainHeight));
    }
  }

  inline void operator() (unsigned int p1, unsigned int p2, unsigned int p3)
  {
    osg::Vec3 v1 = (*vertices)[p1];
    osg::Vec3 v2 = (*vertices)[p2];
    osg::Vec3 v3 = (*vertices)[p3];

    addCurtainIfOnBorder(p1, p2, v1, v2);
    addCurtainIfOnBorder(p2, p3, v2, v3);
    addCurtainIfOnBorder(p3, p1, v3, v1);
  }

  osg::BoundingBox bbox;
  osg::Vec3Array *vertices;
  osg::Vec3Array *normals;
  osg::Vec2Array *texCoords;
  osg::DrawElementsUShort *primitives;
  float curtainHeight;
};

CreateCurtainsVisitor::CreateCurtainsVisitor(const osg::BoundingBox &bbox, const float &curtainHeight)
  : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
  , _bbox(bbox)
  , _curtainHeight(curtainHeight)
{
}

void CreateCurtainsVisitor::apply(osg::Geometry &geometry)
{
  osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(geometry.getVertexArray());
  if (!vertices)
    return;

  osg::Vec3Array *normals = dynamic_cast<osg::Vec3Array *>(geometry.getNormalArray());
  if (!normals)
    return;

  osg::Vec2Array *texCoords = dynamic_cast<osg::Vec2Array *>(geometry.getTexCoordArray(0));
  if (!texCoords)
      return;

  osg::TriangleIndexFunctor<FindEdgesFunctor> findEdgesFunctor;
  findEdgesFunctor.setBbox(_bbox);
  findEdgesFunctor.setCurtainHeight(_curtainHeight);
  findEdgesFunctor.vertices = vertices;
  findEdgesFunctor.normals = normals;
  findEdgesFunctor.texCoords = texCoords;
  geometry.accept(findEdgesFunctor);

  if (findEdgesFunctor.primitives->getNumIndices() > 0)
    geometry.addPrimitiveSet(findEdgesFunctor.primitives);
}
