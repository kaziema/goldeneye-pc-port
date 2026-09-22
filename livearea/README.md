# LiveArea assets — not yet supplied

Need to be added before `make -f Makefile.vita` can produce a VPK:

| File | Spec |
|---|---|
| `icon0.png` | 128x128, 8-bit indexed PNG |
| `bg.png` | 8-bit indexed PNG |
| `startup.png` | 280x158, 8-bit indexed PNG |
| `github.png` | 200x56, RGBA (optional link tile) |

8-bit indexed is required — RGBA fails install with `0x8010113D`.
`check-livearea` in the Makefile catches this before packaging.

`template.xml` is scaffolded (adapted from the Paper Mario Vita port).
Actual GoldenEye art still needs to be dropped in.
