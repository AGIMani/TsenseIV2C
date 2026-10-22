# SDK ↔ firmware compatibility

Host protocol is independent of the C++ SDK patch version. `connectUsb` / `connectEth` send FUN `0x01` and require:

- `proto_major == 1`
- `app_major == 1` (H7 `APP_VERSION`)

| SDK | Host proto | H7 APP_VERSION | Notes |
|-----|------------|----------------|-------|
| 1.1.x | 1.0 | 1.0.0.0+ with FUN 0x01 | Current |
| 1.0.0 | (none) | any AA55 APP | No handshake; do not mix with 1.1.x clients |

## FUN 0x01

Host → H7 (8 B, optional): SDK major/minor/patch, proto major/minor.

H7 → host (8 B):

| Offset | Field |
|--------|--------|
| 0 | proto_major |
| 1 | proto_minor |
| 2 | app_major |
| 3 | app_minor |
| 4 | app_patch |
| 5 | app_build |
| 6 | hand side: 0 unset, 1 right, 2 left |
| 7 | reserved |

Mismatch or no reply → `Err::FwIncompatible` or `Err::FwTimeout`. `attach()` (custom transport) skips the query.

`a3::sdkVersion()` is the host library. `A3HandApi::firmware()` is the H7 APP.
