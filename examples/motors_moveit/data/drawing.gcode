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
G00 F300.0 Z20.000; pen park !!Zpark
G00 F2400.0 Y-20.000; !!Ybottom
G00 F2400.0 X-60.000; !!Xleft
G00 F2400.0 X-59.185 Y19.409; move !!Xleft+0.815 Ybottom+39.409
G00 F300.0 Z0.000; pen down !!Zwork
G01 F2100.0 X59.857 Y19.409; draw !!Xleft+119.857 Ybottom+39.409
G01 F2100.0 X59.857 Y-19.719; draw !!Xleft+119.857 Ybottom+0.281
G01 F2100.0 X-59.185 Y-19.719; draw !!Xleft+0.815 Ybottom+0.281
G01 F2100.0 X-59.185 Y19.409; draw !!Xleft+0.815 Ybottom+39.409
G00 F2400.0 X-14.232 Y0.000; move !!Xleft+45.768 Ybottom+20.000
G01 F2100.0 X-13.959 Y-2.112; draw !!Xleft+46.041 Ybottom+17.888
G01 F2100.0 X-13.149 Y-4.142; draw !!Xleft+46.851 Ybottom+15.858
G01 F2100.0 X-11.834 Y-6.014; draw !!Xleft+48.166 Ybottom+13.986
G01 F2100.0 X-7.907 Y-9.000; draw !!Xleft+52.093 Ybottom+11.000
G01 F2100.0 X-5.446 Y-10.000; draw !!Xleft+54.554 Ybottom+10.000
G01 F2100.0 X-0.000 Y-10.824; draw !!Xleft+60.000 Ybottom+9.176
G01 F2100.0 X5.446 Y-10.000; draw !!Xleft+65.446 Ybottom+10.000
G01 F2100.0 X7.907 Y-9.000; draw !!Xleft+67.907 Ybottom+11.000
G01 F2100.0 X10.064 Y-7.654; draw !!Xleft+70.064 Ybottom+12.346
G01 F2100.0 X13.149 Y-4.142; draw !!Xleft+73.149 Ybottom+15.858
G01 F2100.0 X13.959 Y-2.112; draw !!Xleft+73.959 Ybottom+17.888
G01 F2100.0 X14.232 Y0.000; draw !!Xleft+74.232 Ybottom+20.000
G01 F2100.0 X13.959 Y2.112; draw !!Xleft+73.959 Ybottom+22.112
G01 F2100.0 X13.149 Y4.142; draw !!Xleft+73.149 Ybottom+24.142
G01 F2100.0 X11.834 Y6.014; draw !!Xleft+71.834 Ybottom+26.014
G01 F2100.0 X7.907 Y9.000; draw !!Xleft+67.907 Ybottom+29.000
G01 F2100.0 X5.446 Y10.000; draw !!Xleft+65.446 Ybottom+30.000
G01 F2100.0 X0.000 Y10.824; draw !!Xleft+60.000 Ybottom+30.824
G01 F2100.0 X-5.446 Y10.000; draw !!Xleft+54.554 Ybottom+30.000
G01 F2100.0 X-7.907 Y9.000; draw !!Xleft+52.093 Ybottom+29.000
G01 F2100.0 X-10.064 Y7.654; draw !!Xleft+49.936 Ybottom+27.654
G01 F2100.0 X-13.149 Y4.142; draw !!Xleft+46.851 Ybottom+24.142
G01 F2100.0 X-13.959 Y2.112; draw !!Xleft+46.041 Ybottom+22.112
G01 F2100.0 X-14.232 Y0.000; draw !!Xleft+45.768 Ybottom+20.000
G00 F300.0 Z20.000; pen park !!Zpark
