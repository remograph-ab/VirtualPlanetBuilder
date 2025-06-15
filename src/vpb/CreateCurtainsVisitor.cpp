#include <algorithm>

#include <vpb/CreateCurtainsVisitor>

#include <osg/BoundingBox>
#include <osg/TriangleIndexFunctor>
#include <osg/PrimitiveSet>


#define TOLERANCE 0.001f
#define CURTAIN_RATIO 0.03f

struct FindEdgesFunctor
{
  FindEdgesFunctor()
  : vertices(NULL)
  , primitives(new osg::DrawElementsUShort(osg::PrimitiveSet::QUADS))
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

      primitives->addElement(vertices->size());
      vertices->push_back(osg::Vec3(v1.x(), v1.y(), v1.z() - curtainHeight));

      primitives->addElement(vertices->size());
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

  osg::TriangleIndexFunctor<FindEdgesFunctor> findEdgesFunctor;
  findEdgesFunctor.setBbox(_bbox);
  findEdgesFunctor.setCurtainHeight(_curtainHeight);
  findEdgesFunctor.vertices = vertices;
  geometry.accept(findEdgesFunctor);

  if (findEdgesFunctor.primitives->getNumIndices() > 0)
    geometry.addPrimitiveSet(findEdgesFunctor.primitives);
}
