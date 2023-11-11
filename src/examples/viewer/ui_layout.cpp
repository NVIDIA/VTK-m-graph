// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

static const char *g_defaultLayout =
R"layout(
[Window][MainDockSpace]
Pos=0,32
Size=1920,1048
Collapsed=0

[Window][Graph Controls]
Pos=1370,32
Size=550,432
Collapsed=0
DockId=0x00000003,0

[Window][Viewport]
Pos=0,32
Size=1368,1048
Collapsed=0
DockId=0x00000001,0

[Window][Lights Editor]
Pos=1370,466
Size=550,614
Collapsed=0
DockId=0x00000004,1

[Window][Debug##Default]
Pos=60,60
Size=400,400
Collapsed=0

[Window][TF Editor]
Pos=1370,466
Size=550,614
Collapsed=0
DockId=0x00000004,0

[Docking][Data]
DockSpace     ID=0x782A6D6B Window=0xDEDC5B90 Pos=0,32 Size=1920,1048 Split=X
  DockNode    ID=0x00000001 Parent=0x782A6D6B SizeRef=1368,1048 CentralNode=1 Selected=0x13926F0B
  DockNode    ID=0x00000002 Parent=0x782A6D6B SizeRef=550,1048 Split=Y Selected=0x5098EBE6
    DockNode  ID=0x00000003 Parent=0x00000002 SizeRef=550,432 Selected=0x0D4271D2
    DockNode  ID=0x00000004 Parent=0x00000002 SizeRef=550,614 Selected=0xE3280322
)layout";

const char *getDefaultUILayout()
{
  return g_defaultLayout;
}
