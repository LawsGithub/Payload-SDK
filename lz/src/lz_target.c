/**
 * @file lz_target.c
 * @brief 目标列表的容器操作。
 *
 * 手写动态数组而不是引入第三方容器：lz_core 要保持"零依赖、桌面上
 * clone 下来就能编"的性质（与 wt_inspection 的 wt_core 同约定）。
 */

#include "lz_target.h"

#include <stdlib.h>
#include <string.h>

#define LZ_TARGETLIST_INIT_CAP 16

void LzTargetList_Init(LzTargetList *list)
{
    if (list == NULL) {
        return;
    }
    list->items = NULL;
    list->count = 0;
    list->capacity = 0;
}

LzStatus LzTargetList_Push(LzTargetList *list, const LzTarget *target)
{
    if (list == NULL || target == NULL) {
        return LZ_ERR_PARAM;
    }

    if (list->count == list->capacity) {
        size_t newCap = (list->capacity == 0) ? LZ_TARGETLIST_INIT_CAP : list->capacity * 2;
        LzTarget *grown = realloc(list->items, newCap * sizeof(*grown));
        if (grown == NULL) {
            return LZ_ERR_IO;
        }
        list->items = grown;
        list->capacity = newCap;
    }

    list->items[list->count++] = *target;
    return LZ_OK;
}

void LzTargetList_Free(LzTargetList *list)
{
    if (list == NULL) {
        return;
    }
    free(list->items);
    list->items = NULL;
    list->count = 0;
    list->capacity = 0;
}

bool LzTarget_IsUsable(const LzTarget *target, double minConfidence)
{
    if (target == NULL) {
        return false;
    }
    /* 位置必须合法 —— 这是绕飞的圆心，错了整个圆都错。
     * 杆高与半径允许未知（<= 0）：绕飞只需知道圆心在哪，高度走配置。
     * 把"必须已知"的判据收紧到真正必需的那一项，比一律要求更实用。 */
    if (!LzGeo_IsValid(&target->geo)) {
        return false;
    }
    return target->confidence >= minConfidence;
}
