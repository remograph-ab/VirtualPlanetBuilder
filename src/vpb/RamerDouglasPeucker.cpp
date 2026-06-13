#include <algorithm>
#include <set>
#include <stack>

#include <vpb/RamerDouglasPeucker>

using namespace vpb;

void RamerDouglasPeucker::simplifyBorderVertices(
    osg::HeightField *grid, const unsigned int &numColumns, const unsigned int &numRows,
    std::vector<unsigned int> &simplifiedBottomColumns, std::vector<unsigned int> &simplifiedRightRows,
    std::vector<unsigned int> &simplifiedTopRows, std::vector<unsigned int> &simplifiedLeftRows,
    const float &maximumError
)
{
    if (maximumError == 0.0f) {

        return;
    }

    // Bottom
    std::vector<float> bottomHeights;
    for (unsigned int c = 0; c < numColumns; ++c)
        bottomHeights.push_back(grid->getHeight(c, 0));
    RamerDouglasPeucker::simplifyHeights(bottomHeights, grid->getXInterval(), maximumError, simplifiedBottomColumns);

    // Right
    std::vector<float> rightHeights;
    for (unsigned int r = 0; r < numRows; ++r)
        rightHeights.push_back(grid->getHeight(numColumns - 1, r));
    RamerDouglasPeucker::simplifyHeights(rightHeights, grid->getYInterval(), maximumError, simplifiedRightRows);

    // Top
    std::vector<float> topHeights;
    for (unsigned int c = 0; c < numColumns; ++c)
        topHeights.push_back(grid->getHeight(c, numRows - 1));
    RamerDouglasPeucker::simplifyHeights(topHeights, grid->getXInterval(), maximumError, simplifiedTopRows);

    // Left
    std::vector<float> leftHeights;
    for (unsigned int r = 0; r < numRows; ++r)
        leftHeights.push_back(grid->getHeight(0, r));
    RamerDouglasPeucker::simplifyHeights(leftHeights, grid->getYInterval(), maximumError, simplifiedLeftRows);
}

void RamerDouglasPeucker::simplifyHeights(const std::vector<float>& heights, const float& delta, const float& maxDiff, std::vector<unsigned int> &decimatedIndices)
{
    decimatedIndices.clear();
    if (heights.size() < 2)
        return;

    // Run RDP iteratively and collect indices
    rdpIterative(heights, delta, maxDiff, decimatedIndices);

    // Sort indices to maintain original order
    std::sort(decimatedIndices.begin(), decimatedIndices.end());
}

void RamerDouglasPeucker::rdpIterative(
  const std::vector<float>& heights, const float &delta, const float &maxDiff,
  std::vector<unsigned int> &outIndices
)
{
  struct Segment { unsigned int start; unsigned int end; };
  std::stack<Segment> segmentStack;

  segmentStack.push({ 0, static_cast<unsigned int>(heights.size() - 1) });
  std::set<unsigned int> markedIndices;

  while (!segmentStack.empty()) {
    Segment current = segmentStack.top();
    segmentStack.pop();

    float maxDistance = 0.0f;
    int indexFurthest = -1;

    // Find furthest point
    for (unsigned int i = current.start + 1; i < current.end; ++i) {
      float dist = perpendicularDistance(
        i, current.start, current.end,
        heights[i], heights[current.start], heights[current.end], delta
      );
      if (dist > maxDistance) {
        maxDistance = dist;
        indexFurthest = i;
      }
    }

    if (maxDistance > maxDiff && indexFurthest > -1) {
      // Push segments to process (in reverse order for correct processing)
      segmentStack.push({ static_cast<unsigned int>(indexFurthest), current.end });
      segmentStack.push({ current.start, static_cast<unsigned int>(indexFurthest) });
    }
    else {
      // Mark endpoints as kept
      markedIndices.insert(current.start);
      markedIndices.insert(current.end);
    }
  }

  // Convert set to vector, maintaining order
  for (unsigned int i = 0; i < heights.size(); ++i) {
    if (markedIndices.count(i)) {
      outIndices.push_back(i);
    }
  }
}

float RamerDouglasPeucker::perpendicularDistance(
    const unsigned int &index, const unsigned int &startIdx, const unsigned int &endIdx,
    const float &height, const float &startHeight, const float &endHeight, const float &delta
)
{
    // Convert index to x-coordinates using delta
    float x = index * delta;
    float x1 = startIdx * delta;
    float x2 = endIdx * delta;

    // Line equation: from (x1, y1) to (x2, y2)
    float y = height;
    float y1 = startHeight;
    float y2 = endHeight;

    float numerator = std::abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1);
    float denominator = std::hypot(x2 - x1, y2 - y1);
    return denominator > 0.0f ? numerator / denominator : 0.0f;
}
