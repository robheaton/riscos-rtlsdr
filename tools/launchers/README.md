# Obey launchers

RISC OS Obey files (filetype `feb`, hence the `,feb` suffix) that set the
`RTLSDRView$...` tunables documented in the top-level README and then start
`RTLSDRView` from the same directory -- copy them next to the built
`RTLSDRView,ff8`. Each writes a per-second trace to a `LogN` file next to
itself when the app's window is closed.

| launcher | rate | read size | buffer | lead | tuner pause | what it is |
|---|---|---|---|---|---|---|
| `T1_240k_96K` | 240 kSPS | 96 KB | 512 KB | 300 ms | 10 ms | first good profile: lossless, but ~1 s stalls still underrun |
| `T2_2400k_16K` | 2.4 MSPS | 16 KB | 512 KB | 350 ms | 10 ms | the old behaviour (baseline) |
| `T3_2400k_128K` | 2.4 MSPS | 128 KB | 512 KB | 350 ms | 10 ms | wide spectrum, big reads |
| `T4_240k_16K` | 240 kSPS | 16 KB | 128 KB | 300 ms | 10 ms | low rate, small reads |
| `T5_240k_192K` | 240 kSPS | 192 KB | 1024 KB | 500 ms | 10 ms | the "sweet spot" -- now the built-in defaults (except the tuner pause, now 0) |
| `T6_240k_96K_pace0` | 240 kSPS | 96 KB | 512 KB | 300 ms | 0 | retunes in ~50 ms instead of 200-600 ms |
| `Run_defaults_with_log` | (defaults) | | | | | just the defaults, with a log |

Always run directly from the Filer, never inside a TaskWindow.
