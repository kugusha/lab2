#include "astar.h"
#include <math.h>

Astar::Astar(double HW, bool BT)
{
    hweight = HW;
    breakingties = BT;
}

double Astar::computeHFromCellToCell(int i1, int j1, int i2, int j2, const EnvironmentOptions &options)
{
    auto chosen_metric = options.metrictype;
    int dx = abs(i2 - i1);
    int dy = abs(j2 - j1);

    if (chosen_metric == CN_SP_MT_EUCL) {
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    if (chosen_metric == CN_SP_MT_DIAG) {
        auto res = dx + dy - 2 * std::min(dx, dy) + sqrt(2) * std::min(dx, dy);
        return res;
    }

    if (chosen_metric == CN_SP_MT_MANH) {
        return dx + dy;
    }

    if (chosen_metric == CN_SP_MT_CHEB) {
        return dx + dy - std::min(dx, dy);
    }

    return 0;
}
