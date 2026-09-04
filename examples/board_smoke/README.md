# board_smoke (quarantined)

`main.cpp` is the original WIZnet + FreeRTOS board bring-up smoke (NTP sync,
DHCP/static/SLAAC, custom transport over the WIZnet socket). It does not build
yet — it predates the `rosette::App` model and needs the board transport path
reworked (refactor phases 4/7). Kept as the reference for that work; not wired
into the build.
