/* Vita build only: the 32-bit sidecar layout assumes these struct sizes. */
#if defined(__vita__)
#include <ultra64.h>
#include <bondtypes.h>
#include "bg.h"

_Static_assert(sizeof(bg_portal_data_entry) == 8, "bg portal entry");
_Static_assert(sizeof(stagesetup) == 40, "stagesetup");
_Static_assert(sizeof(PadRecord) == 44, "PadRecord");
_Static_assert(sizeof(BoundPadRecord) == 68, "BoundPadRecord");
_Static_assert(sizeof(waypoint) == 16, "waypoint");
_Static_assert(sizeof(waygroup) == 12, "waygroup");
_Static_assert(sizeof(PathRecord) == 8, "PathRecord");
_Static_assert(sizeof(AIListRecord) == 8, "AIListRecord");
_Static_assert(sizeof(ObjectRecord) == 128, "ObjectRecord");
_Static_assert(sizeof(DoorRecord) == 256, "DoorRecord");
_Static_assert(sizeof(KeyRecord) == 132, "KeyRecord");
_Static_assert(sizeof(GuardRecord) == 28, "GuardRecord");
_Static_assert(sizeof(SetupIntroCamera) == 40, "SetupIntroCamera");
_Static_assert(sizeof(MonitorObjRecord) == 256, "MonitorObjRecord");
_Static_assert(sizeof(MultiMonitorObjRecord) == 596, "MultiMonitorObjRecord");
_Static_assert(sizeof(VehichleRecord) == 176, "VehichleRecord");
_Static_assert(sizeof(TankRecord) == 224, "TankRecord");
#endif
