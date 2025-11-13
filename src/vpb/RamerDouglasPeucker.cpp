#include <algorithm>

#include <vpb/RamerDouglasPeucker>

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

    // Run RDP recursively and collect indices
    rdpRecursive(heights, delta, maxDiff, 0, static_cast<unsigned int>(heights.size() - 1), decimatedIndices);

    // Sort indices to maintain original order
    std::sort(decimatedIndices.begin(), decimatedIndices.end());
}

void RamerDouglasPeucker::rdpRecursive(
    const std::vector<float>& heights, const float &delta, const float &maxDiff,
    const unsigned int &startIdx, const unsigned int &endIdx, std::vector<unsigned int> &outIndices
)
{
    float maxDistance = 0.0f;
    int indexFurthest = -1;
    for (unsigned int i = startIdx + 1; i <= endIdx; ++i) {
        float dist = perpendicularDistance(i, startIdx, endIdx, heights[i], heights[startIdx], heights[endIdx], delta);
        if (dist > maxDistance) {
            maxDistance = dist;
            indexFurthest = i;
        }
    }

    if (maxDistance > maxDiff && indexFurthest > -1) {
        // Recursive simplification
        rdpRecursive(heights, delta, maxDiff, startIdx, static_cast<unsigned int>(indexFurthest), outIndices);
        rdpRecursive(heights, delta, maxDiff, static_cast<unsigned int>(indexFurthest), endIdx, outIndices);
    }
    else {
        // Keep start and end indices (startIdx added in parent call)
        if (std::find(outIndices.begin(), outIndices.end(), startIdx) == outIndices.end())
            outIndices.push_back(startIdx);
        if (std::find(outIndices.begin(), outIndices.end(), endIdx) == outIndices.end())
            outIndices.push_back(endIdx);
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
