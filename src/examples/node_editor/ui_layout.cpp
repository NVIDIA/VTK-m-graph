// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

static const char *g_defaultLayout =
R"layout(
[Window][MainDockSpace]
Pos=0,32
Size=3138,1603
Collapsed=0

[Window][Viewport]
Pos=0,32
Size=2618,1076
Collapsed=0
DockId=0x00000005,0

[Window][Debug##Default]
Pos=60,60
Size=400,400
Collapsed=0

[Window][NodeEditor]
Pos=0,1110
Size=2618,525
Collapsed=0
DockId=0x00000006,0

[Window][Lights Editor]
Pos=2620,32
Size=518,523
Collapsed=0
DockId=0x00000003,0

[Window][Node Info]
Pos=2620,557
Size=518,1078
Collapsed=0
DockId=0x00000004,0

[Docking][Data]
DockSpace     ID=0x782A6D6B Pos=0,32 Size=1920,1048 CentralNode=1 Selected=0x13926F0B
DockSpace     ID=0x80F5B4C5 Window=0x079D3A04 Pos=0,32 Size=3138,1603 Split=X
  DockNode    ID=0x00000001 Parent=0x80F5B4C5 SizeRef=2618,1603 Split=Y
    DockNode  ID=0x00000005 Parent=0x00000001 SizeRef=2618,1076 CentralNode=1 Selected=0xC450F867
    DockNode  ID=0x00000006 Parent=0x00000001 SizeRef=2618,525 Selected=0xD009B1B2
  DockNode    ID=0x00000002 Parent=0x80F5B4C5 SizeRef=518,1603 Split=Y Selected=0x7ECBF265
    DockNode  ID=0x00000003 Parent=0x00000002 SizeRef=518,523 Selected=0x9BF64AD9
    DockNode  ID=0x00000004 Parent=0x00000002 SizeRef=518,1078 Selected=0x7ECBF265
)layout";

const char *getDefaultUILayout()
{
  return g_defaultLayout;
}
