A3.2 original CAD reference

This directory preserves the original left/right URDF files and all 44 STL
meshes referenced by them, plus the upstream SDK license. The bytes were copied
from the original SDK CAD source and verified against the model provenance
hashes before packaging. File hashes are recorded in
migration/ubuntu/vendor-cad-source.json.

The browser uses the reviewed, derived models in assets/hands. These original
CAD files support retained model preparation and independent provenance tests.
No prebuilt SDK executable, Python environment or historical CAD backup is
included here.

tools/prepare_hand_v3_2.py and prepare_hand_v3_2_withid.py use this directory by
default. A32_SDK_ROOT or the with-ID tool's --sdk-root can override it explicitly.
The native Linux hardware backend is in src/a32_driver/native_linux and
src/a32_driver/bin/liba32_native.so; it does not use a Windows SDK binary.

2026-10-10 SDK update: this directory now also contains the user-supplied
SDK 1.1.14 sources. sdk-source.json records the archive and exact upstream file
hashes; dist prebuilt artifacts are excluded. The original CAD references above
remain unchanged. The native Linux adapter statically links the SDK protocol
and conversion helpers while retaining project transport and stop behavior.
