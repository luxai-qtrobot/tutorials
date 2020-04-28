G00 S1; endstops
G00 E0; no extrusion
G01 S1; endstops
G01 E0; no extrusion
G21; millimeters
G91 G0 F300.0 Z20.000; pen park !!Zsafe
G90; absolute
G28 X; home
G28 Y; home
G28 Z; home
G00 F300.0 Z30.000; pen park !!Zpark
G00 F2400.0 Y-20.000; !!Ybottom
G00 F2400.0 X-60.000; !!Xleft
G00 F2400.0 X21.287 Y-12.025; move !!Xleft+81.287 Ybottom+7.975
G00 F300.0 Z10.000; pen down !!Zwork
G01 F2100.0 X18.204 Y-9.896; draw !!Xleft+78.204 Ybottom+10.104
G01 F2100.0 X15.951 Y-6.847; draw !!Xleft+75.951 Ybottom+13.153
G01 F2100.0 X14.559 Y-2.983; draw !!Xleft+74.559 Ybottom+17.017
G01 F2100.0 X14.095 Y1.592; draw !!Xleft+74.095 Ybottom+21.592
G01 F2100.0 X14.303 Y4.778; draw !!Xleft+74.303 Ybottom+24.778
G01 F2100.0 X15.964 Y10.126; draw !!Xleft+75.964 Ybottom+30.126
G01 F2100.0 X17.418 Y12.288; draw !!Xleft+77.418 Ybottom+32.288
G01 F2100.0 X19.218 Y14.038; draw !!Xleft+79.218 Ybottom+34.038
G01 F2100.0 X21.296 Y15.289; draw !!Xleft+81.296 Ybottom+35.289
G01 F2100.0 X23.653 Y16.039; draw !!Xleft+83.653 Ybottom+36.039
G01 F2100.0 X26.288 Y16.289; draw !!Xleft+86.288 Ybottom+36.289
G01 F2100.0 X28.932 Y16.039; draw !!Xleft+88.932 Ybottom+36.039
G01 F2100.0 X31.298 Y15.289; draw !!Xleft+91.298 Ybottom+35.289
G01 F2100.0 X33.385 Y14.038; draw !!Xleft+93.385 Ybottom+34.038
G01 F2100.0 X35.194 Y12.288; draw !!Xleft+95.194 Ybottom+32.288
G01 F2100.0 X36.648 Y10.126; draw !!Xleft+96.648 Ybottom+30.126
G01 F2100.0 X37.686 Y7.623; draw !!Xleft+97.686 Ybottom+27.623
G01 F2100.0 X38.517 Y1.592; draw !!Xleft+98.517 Ybottom+21.592
G01 F2100.0 X38.309 Y-1.587; draw !!Xleft+98.309 Ybottom+18.413
G01 F2100.0 X36.648 Y-6.935; draw !!Xleft+96.648 Ybottom+13.065
G01 F2100.0 X35.194 Y-9.104; draw !!Xleft+95.194 Ybottom+10.896
G01 F2100.0 X33.385 Y-10.847; draw !!Xleft+93.385 Ybottom+9.153
G01 F2100.0 X31.298 Y-12.091; draw !!Xleft+91.298 Ybottom+7.909
G01 F2100.0 X28.932 Y-12.838; draw !!Xleft+88.932 Ybottom+7.162
G01 F2100.0 X26.288 Y-13.087; draw !!Xleft+86.288 Ybottom+6.913
G01 F2100.0 X23.112 Y-12.717; draw !!Xleft+83.112 Ybottom+7.283
G01 F2100.0 X20.560 Y-14.059; draw !!Xleft+80.560 Ybottom+5.941
G01 F2100.0 X17.877 Y-16.344; draw !!Xleft+77.877 Ybottom+3.656
G01 F2100.0 X17.203 Y-16.924; draw !!Xleft+77.203 Ybottom+3.076
G01 F2100.0 X21.287 Y-12.025; draw !!Xleft+81.287 Ybottom+7.975
G00 F300.0 Z14.000; pen up !!Zup
G00 F2400.0 X12.968 Y14.994; move !!Xleft+72.968 Ybottom+34.994
G00 F300.0 Z10.000; pen down !!Zwork
G01 F2100.0 X-10.979 Y14.994; draw !!Xleft+49.021 Ybottom+34.994
G00 F300.0 Z14.000; pen up !!Zup
G00 F2400.0 X0.923 Y14.923; move !!Xleft+60.923 Ybottom+34.923
G00 F300.0 Z10.000; pen down !!Zwork
G01 F2100.0 X0.923 Y-16.080; draw !!Xleft+60.923 Ybottom+3.920
G00 F300.0 Z14.000; pen up !!Zup
G00 F2400.0 X-38.868 Y-13.799; move !!Xleft+21.132 Ybottom+6.201
G00 F300.0 Z10.000; pen down !!Zwork
G01 F2100.0 X-39.369 Y-13.344; draw !!Xleft+20.631 Ybottom+6.656
G01 F2100.0 X-46.052 Y-7.211; draw !!Xleft+13.948 Ybottom+12.789
G01 F2100.0 X-50.254 Y-2.622; draw !!Xleft+9.746 Ybottom+17.378
G01 F2100.0 X-52.437 Y1.200; draw !!Xleft+7.563 Ybottom+21.200
G01 F2100.0 X-53.062 Y5.030; draw !!Xleft+6.938 Ybottom+25.030
G01 F2100.0 X-52.475 Y8.228; draw !!Xleft+7.525 Ybottom+28.228
G01 F2100.0 X-50.870 Y10.829; draw !!Xleft+9.130 Ybottom+30.829
G01 F2100.0 X-48.483 Y12.577; draw !!Xleft+11.517 Ybottom+32.577
G01 F2100.0 X-45.547 Y13.217; draw !!Xleft+14.453 Ybottom+33.217
G01 F2100.0 X-43.247 Y12.457; draw !!Xleft+16.753 Ybottom+32.457
G01 F2100.0 X-41.425 Y10.664; draw !!Xleft+18.575 Ybottom+30.664
G01 F2100.0 X-38.868 Y6.883; draw !!Xleft+21.132 Ybottom+26.883
G01 F2100.0 X-36.311 Y10.664; draw !!Xleft+23.689 Ybottom+30.664
G01 F2100.0 X-34.488 Y12.457; draw !!Xleft+25.512 Ybottom+32.457
G01 F2100.0 X-32.188 Y13.217; draw !!Xleft+27.812 Ybottom+33.217
G01 F2100.0 X-29.253 Y12.577; draw !!Xleft+30.747 Ybottom+32.577
G01 F2100.0 X-26.865 Y10.829; draw !!Xleft+33.135 Ybottom+30.829
G01 F2100.0 X-25.260 Y8.228; draw !!Xleft+34.740 Ybottom+28.228
G01 F2100.0 X-24.673 Y5.030; draw !!Xleft+35.327 Ybottom+25.030
G01 F2100.0 X-25.298 Y1.200; draw !!Xleft+34.702 Ybottom+21.200
G01 F2100.0 X-27.481 Y-2.622; draw !!Xleft+32.519 Ybottom+17.378
G01 F2100.0 X-31.683 Y-7.211; draw !!Xleft+28.317 Ybottom+12.789
G01 F2100.0 X-38.868 Y-13.799; draw !!Xleft+21.132 Ybottom+6.201
G00 F300.0 Z14.000; pen up !!Zup
G00 F2400.0 X-58.992 Y18.503; move !!Xleft+1.008 Ybottom+38.503
G00 F300.0 Z10.000; pen down !!Zwork
G01 F2100.0 X-58.992 Y-18.992; draw !!Xleft+1.008 Ybottom+1.008
G01 F2100.0 X57.525 Y-18.992; draw !!Xleft+117.525 Ybottom+1.008
G01 F2100.0 X57.525 Y18.503; draw !!Xleft+117.525 Ybottom+38.503
G01 F2100.0 X-58.992 Y18.503; draw !!Xleft+1.008 Ybottom+38.503
G00 F300.0 Z30.000; pen park !!Zpark
