# Vendored third-party — Mongoose

The CPCU Dashboard's web bridge (`cpcu_ws`) uses
[Mongoose](https://github.com/cesanta/mongoose) as its embedded
HTTP/WebSocket server. Mongoose is a single-file C library with a
permissive license (dual GPLv2 / commercial; we use it under GPLv2
for non-commercial academic work, which a senior capstone clearly
qualifies as).

## Why vendored, not apt-installed

- Mongoose isn't in Raspberry Pi OS apt repositories.
- It's literally one `.c` and one `.h` — vendoring is the path of
  least friction.
- Pinning to a known commit means the CPCU build is reproducible
  even if upstream breaks.

## Files (not present until you run `fetch.sh`)

- `mongoose.c` — single-file implementation (~10K LOC at v7.14).
- `mongoose.h` — public API header.

These are intentionally **not** committed to the repository. They're
fetched on-demand. The `cpcu_ws` CMake target gracefully skips the
build if the files are absent, so a fresh clone on a network-isolated
box still configures cleanly.

## Fetching

From this directory:

```sh
./fetch.sh
```

Or manually:

```sh
curl -fsSL -o mongoose.h \
    https://raw.githubusercontent.com/cesanta/mongoose/7.14/mongoose.h
curl -fsSL -o mongoose.c \
    https://raw.githubusercontent.com/cesanta/mongoose/7.14/mongoose.c
```

After fetching, re-run `cmake -S . -B build` from the project root.
The `cpcu_ws` target will appear and build.

## Pinned version

We target Mongoose **7.14** (released October 2024). Newer releases
should also work — Mongoose's API is very stable — but 7.14 is what
the bridge code is tested against. To change the pinned version,
edit `fetch.sh` and `WEB_DASHBOARD.md`.

## What we use from Mongoose

A small surface area:

| API | Purpose |
|---|---|
| `mg_mgr_init` / `mg_mgr_free` / `mg_mgr_poll` | event loop |
| `mg_http_listen` | HTTP/WS listener on `:8765` |
| `mg_http_serve_dir` | serve `static/` |
| `mg_ws_upgrade` | upgrade `/ws` to a WebSocket |
| `mg_ws_send` | broadcast frames to a client |
| `MG_EV_*` events | connection lifecycle |

We do not use Mongoose's TLS, MQTT, DNS, or filesystem layers.

## License notes

Mongoose is dual-licensed. We use the GPLv2 branch. Academic
research/teaching at a university qualifies under GPLv2. If this
project is ever re-licensed for commercial use, the bridge would
need either a commercial Mongoose license or migration to a
permissively-licensed alternative (libwebsockets is LGPLv2.1).
