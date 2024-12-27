#ifndef CONSTANTS
#define CONSTANTS

//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//    X-COORDINATE => designating the width of the grid
//    Y-COORDINATE => designating the height of the grid
/**
 * 约定: 
 *     HEADING： [0, 359]度，0表示朝向北(指向Y)
 *     X - 表示网格的宽度
 *     Y - 表示网格的高度
 */
#include <cmath>

namespace HybridAStar 
{
namespace Constants {

static const float r = 2.0;//最小转弯半径
static const int headings = 36;
static const bool usedubins = true; 

/**
 * 如果cost-so-far的启发式值比cost-to-come的启发式值更大时，算法应选择predecessor而不是successor。
 * 这样会导致successor从不会被选择的情况发生，该单元格永远只能扩展一个节点。tieBreaker可以人为地增加
 * predecessor的代价，允许successor放置在同一个单元中。它的使用见algorithm.cpp, 
 *     if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) 
 */
static const float tieBreaker = 0.95;//0.25*0.84(一段弧线的距离)
static const float penaltyTurning = 1.05;
static const float penaltyReversing = 2.0;
static const float penaltyCOD = 2.5;
static const float dubinsShotDistance = 100;
static const float dubinsStepSize = 0.25;
static const float max_acc = 1.5;
static const float max_vel = 2.0;//这里和优化过程中不一致
static const float ts = 0.25;
}
}

#endif // CONSTANTS

