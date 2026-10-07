A3.2 original CAD reference

This directory preserves the original left/right URDF files and all 44 STL
meshes referenced by them, plus the upstream SDK license. The bytes were copied
from the existing D:/Qtworks/a3-cpp-sdk source and verified against the model
provenance hashes before packaging. File hashes are recorded in
migration/ubuntu/vendor-cad-source.json.

The browser uses the reviewed, derived models in assets/hands. These original
CAD files support retained model preparation and independent provenance tests.
No SDK executable, Windows bridge, Python environment or historical CAD backup
is included here.

tools/prepare_hand_v3_2.py and prepare_hand_v3_2_withid.py use this directory by
default. A32_SDK_ROOT or the with-ID tool's --sdk-root can override it explicitly.
The native Linux hardware backend is in src/a32_driver/native_linux and
src/a32_driver/bin/liba32_native.so; it does not use a Windows SDK binary.
