// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

static const char *g_defaultLayout =
R"layout(
[Window][MainDockSpace]
Pos=0,32
Size=1920,1048
Collapsed=0

[Window][Viewport]
Pos=0,32
Size=1400,521
Collapsed=0
DockId=0x00000001,0

[Window][Debug##Default]
Pos=60,60
Size=400,400
Collapsed=0

[Window][NodeEditor]
Pos=0,555
Size=1400,525
Collapsed=0
DockId=0x00000002,0

[Window][Lights Editor]
Pos=1402,32
Size=518,524
Collapsed=0
DockId=0x00000005,0

[Window][Node Info]
Pos=1402,558
Size=518,522
Collapsed=0
DockId=0x00000006,0

[Docking][Data]
DockSpace     ID=0x782A6D6B Window=0xDEDC5B90 Pos=0,32 Size=1920,1048 Split=X Selected=0x13926F0B
  DockNode    ID=0x00000003 Parent=0x782A6D6B SizeRef=1400,1048 Split=Y
    DockNode  ID=0x00000001 Parent=0x00000003 SizeRef=1920,527 CentralNode=1 Selected=0x13926F0B
    DockNode  ID=0x00000002 Parent=0x00000003 SizeRef=1920,525 Selected=0x04603C50
  DockNode    ID=0x00000004 Parent=0x782A6D6B SizeRef=518,1048 Split=Y Selected=0x5098EBE6
    DockNode  ID=0x00000005 Parent=0x00000004 SizeRef=518,524 Selected=0x5098EBE6
    DockNode  ID=0x00000006 Parent=0x00000004 SizeRef=518,522 Selected=0xC7FDBA2C
)layout";

const char *getDefaultUILayout()
{
  return g_defaultLayout;
}
