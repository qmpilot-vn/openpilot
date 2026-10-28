# qmpilot-connect on Comma 4

Live host: `https://qmpilot-connect.com`. No trailing slash.

Branch: `vf-dev-c4-mrr-closing`. Upload is implemented. Pairing is not. A car that already has `ApiHost` and `QmpilotApiKey` can upload. A new car cannot get those two values from the server until the pair screen exists.

The Device **Upload** control is one choice. **comma** or **qmpilot**. Default is **comma**. Pairing must set the control to **qmpilot**, or the new key sits unused.

## Do not change

- Keep `/persist/comma/dongle_id` and `/data/params/d/DongleId`. The server does not mint an id.
- Sign with `/persist/comma/id_rsa` (RS256) or `id_ecdsa` (ES256). Send the matching `.pub` file as `public_key`. `BaseApi.get_key_pair()` already reads these.
- Do not call `POST /v2/pilotauth/` from the pair screen. `system/athena/registration.py` still has that call, and it runs only when `DongleId` is missing. A Comma 4 that already has the persist id does not hit it.
- Do not put the API key in the QR.

## Pair

All pair calls go to `https://qmpilot-connect.com`, even before `ApiHost` is set. The car needs Wi-Fi. The JWT includes `identity` (the dongle id), `iat`, `nbf`, and `exp`, lifetime at most 1 hour. `BaseApi.get_token()` already builds that token. Send `Authorization: JWT <token>`.

`Api()` points at `https://api.commadotai.com`. The pair screen must construct `BaseApi(dongle_id, "https://qmpilot-connect.com")` itself.

1. `POST /v1/pair/start` with `{ "public_key": "<contents of the .pub file>" }`.
2. On 200, read `pair_nonce` and `expires_in` (300 seconds). Draw this QR:

```
https://qmpilot-connect.com/pair?dongle_id=<16 hex>&pair_nonce=<pair_nonce>
```

3. Show the dongle id in large text. Call `pair/start` again before the nonce expires and replace the QR.
4. Poll `GET /v1/pair/status` every 2 seconds with the same JWT.
5. When `api_key` is a string, store it and stop polling:
   - `ApiHost` = `api_host` from the response, no trailing slash
   - `QmpilotApiKey` = `api_key`
   - `UploadBackend` = `1` (the qmpilot side of the Device control)
6. Write `ApiHost` and `QmpilotApiKey` with Params, and also as files under `/persist/qmpilot/`, so a manager reset does not delete them. The uploader already reads that directory.
7. The qmpilot uploader reads host and key only at process start. Setting `UploadBackend` to `1` makes manager start it.

`api_key` is null on every later poll. A 409 means this dongle is already registered to another key. A 401 means the JWT or public key was rejected.

The current **Pair Device** button opens `https://connect.comma.ai/?pair=<jwt>`. That is comma connect. It is not this screen.

## Upload

Implemented in `system/loggerd/qmpilot_uploader.py`. It runs only when the Device control is **qmpilot** and both `ApiHost` and `QmpilotApiKey` are set. The comma uploader in `system/loggerd/uploader.py` always talks to `https://api.commadotai.com`, and it stops while the control is **qmpilot**.

| Step | Contract | Firmware |
|---|---|---|
| Ask for a URL | `GET /v1.4/{dongle_id}/upload_url/?path=<log_id>/<seg>/<file>&access_token=<QmpilotApiKey>` and header `X-Device-Key` | yes |
| Send the bytes | PUT `url` using `headers` from the JSON | yes |
| Index the file | `POST /v1/upload/callback` with `{ "dongle_id", "path" }` and `X-Device-Key` | yes, and a callback that is not 200 leaves the file unmarked |
| Path | local `…--<hash>--0/qlog.zst` becomes `…--<hash>/0/qlog.zst` | yes |
| Done | 200, 201, 412 | 200 and 201 are done after callback 200. 412 is done only after that same callback |
| Retry | 401, 403, 503 | unmarked. 503 backs off with `Retry-After`, default 60 s, and does not block the next qlog |
| Not Saved | 503 on `rlog` or a camera | the file stays local |
| Boot and crash logs | not in this contract | not sent |

## Checklist

- [x] Dongle id is the comma id already on the device.
- [x] JWT signer is the persist comma key. Lifetime is 1 hour.
- [x] Upload URL, PUT, callback, and the segment path rewrite.
- [x] 401 and 403 are retried. 503 on a heavy file waits for Save.
- [x] One upload target at a time, chosen under Settings, Device, Upload.
- [x] Pair screen on Settings, Device, **Pair qmpilot**: `POST /v1/pair/start`, QR, poll `GET /v1/pair/status`.
- [x] On a returned key, write Params, write `/persist/qmpilot/`, set Upload to qmpilot, and let manager start the uploader.
