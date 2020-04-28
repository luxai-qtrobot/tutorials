(Scribbled version of /tmp/ink_ext_XXXXXX.svgIA4HJ0 @ 3500.00)
( unicorn.py --tab="homing" --pen-up-angle=90 --pen-down-angle=90 --start-delay=150 --stop-delay=150 --xy-feedrate=3500 --z-feedrate=150 --z-height=0 --finished-height=0 --register-pen=false --x-home=0 --y-home=0 --num-copies=1 --continuous=false --pause-on-layer-change=false /tmp/ink_ext_XXXXXX.svgIA4HJ0 )
G21 (metric ftw)
G90 (absolute mode)
G92 X0.00 Y0.00 Z0.00 (you are here)

(Polyline consisting of 1 segments.)
G1 X-50.59 Y15.35 F3500.00
M300 S90.00 (pen down)
G4 P150 (wait 150ms)
G1 X55.17 Y15.35 F3500.00
G1 X55.17 Y-16.86 F3500.00
G1 X-50.59 Y-16.86 F3500.00
G1 X-50.59 Y15.35 F3500.00
M300 S90.00 (pen up)
G4 P150 (wait 150ms)

(Polyline consisting of 1 segments.)
G1 X-46.32 Y11.64 F3500.00
M300 S90.00 (pen down)
G4 P150 (wait 150ms)
G1 X49.47 Y11.64 F3500.00
G1 X49.47 Y-13.16 F3500.00
G1 X-46.32 Y-13.16 F3500.00
G1 X-46.32 Y11.64 F3500.00
M300 S90.00 (pen up)
G4 P150 (wait 150ms)

(Polyline consisting of 1 segments.)
G1 X-42.32 Y7.94 F3500.00
M300 S90.00 (pen down)
G4 P150 (wait 150ms)
G1 X45.20 Y7.94 F3500.00
G1 X45.20 Y-9.74 F3500.00
G1 X-42.32 Y-9.74 F3500.00
G1 X-42.32 Y7.94 F3500.00
M300 S90.00 (pen up)
G4 P150 (wait 150ms)

(Polyline consisting of 1 segments.)
G1 X-34.63 Y3.66 F3500.00
M300 S90.00 (pen down)
G4 P150 (wait 150ms)
G1 X37.78 Y3.66 F3500.00
G1 X37.78 Y-5.18 F3500.00
G1 X-34.63 Y-5.18 F3500.00
G1 X-34.63 Y3.66 F3500.00
M300 S90.00 (pen up)
G4 P150 (wait 150ms)

(Polyline consisting of 1 segments.)
G1 X-3.55 Y1.38 F3500.00
M300 S90.00 (pen down)
G4 P150 (wait 150ms)
G1 X10.99 Y1.38 F3500.00
G1 X10.99 Y-2.32 F3500.00
G1 X-3.55 Y-2.32 F3500.00
G1 X-3.55 Y1.38 F3500.00
M300 S90.00 (pen up)
G4 P150 (wait 150ms)


(end of print job)
M300 S90.00 (pen up)
G4 P150 (wait 150ms)
M300 S255 (turn off servo)
G1 X0 Y0 F3500.00
G1 Z0.00 F150.00 (go up to finished level)
G1 X0.00 Y0.00 F3500.00 (go home)
M18 (drives off)
