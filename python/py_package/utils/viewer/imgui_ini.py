#
# Copyright 2025 Hillbot Inc.
# Copyright 2020-2024 UCSD SU Lab
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
imgui_ini = """[Window][Control##]
Pos=0,29
Size=398,526
Collapsed=0
DockId=0x00000004,0

[Window][Scene##]
Pos=0,557
Size=398,523
Collapsed=0
DockId=0x00000001,0

[Window][Entity##]
Pos=1520,29
Size=400,526
Collapsed=0
DockId=0x00000007,0

[Window][Articulation##]
Pos=1520,557
Size=400,523
Collapsed=0
DockId=0x00000008,0

[Window][Transform##]
Pos=0,557
Size=398,523
Collapsed=0
DockId=0x00000001,1

[Window][Render##]
Pos=0,29
Size=398,526
Collapsed=0
DockId=0x00000004,1

[Window][DockSpace]
Pos=0,0
Size=1920,1080
Collapsed=0

[Window][Debug##Default]
Pos=60,60
Size=400,400
Collapsed=0

[Docking][Data]
DockSpace       ID=0x4BBE4C7A Pos=0,29 Size=1920,1051 CentralNode=1
DockSpace       ID=0xFE0324EA Window=0x9A404470 Pos=0,29 Size=1920,1051 Split=X
  DockNode      ID=0x00000005 Parent=0xFE0324EA SizeRef=1518,1051 Split=X
    DockNode    ID=0x00000002 Parent=0x00000005 SizeRef=398,1051 Split=Y Selected=0x9CA6559F
      DockNode  ID=0x00000004 Parent=0x00000002 SizeRef=398,526 Selected=0x9CA6559F
      DockNode  ID=0x00000001 Parent=0x00000002 SizeRef=398,523 Selected=0x15CB5A92
    DockNode    ID=0x00000003 Parent=0x00000005 SizeRef=1520,1051 CentralNode=1
  DockNode      ID=0x00000006 Parent=0xFE0324EA SizeRef=400,1051 Split=Y Selected=0x20E45936
    DockNode    ID=0x00000007 Parent=0x00000006 SizeRef=400,526 Selected=0x20E45936
    DockNode    ID=0x00000008 Parent=0x00000006 SizeRef=400,523 Selected=0x24C99AC4
"""
