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
G00 F2400.0 X49.359 Y15.069; move !!Xleft+109.359 Ybottom+35.069
G00 F300.0 Z10.000; pen down !!Zwork
G01 F2100.0 X49.359 Y-14.773; draw !!Xleft+109.359 Ybottom+5.227
G01 F2100.0 X31.847 Y-14.773; draw !!Xleft+91.847 Ybottom+5.227
G01 F2100.0 X31.847 Y-8.957; draw !!Xleft+91.847 Ybottom+11.043
G01 F2100.0 X43.005 Y-8.957; draw !!Xleft+103.005 Ybottom+11.043
G01 F2100.0 X43.005 Y15.069; draw !!Xleft+103.005 Ybottom+35.069
G01 F2100.0 X49.359 Y15.069; draw !!Xleft+109.359 Ybottom+35.069
G00 F300.0 Z14.000; pen up !!Zup
G00 F2400.0 X28.299 Y-6.058; move !!Xleft+88.299 Ybottom+13.942
G00 F300.0 Z10.000; pen down !!Zwork
G01 F2100.0 X28.195 Y-8.192; draw !!Xleft+88.195 Ybottom+11.808
G01 F2100.0 X27.882 Y-10.056; draw !!Xleft+87.882 Ybottom+9.944
G01 F2100.0 X27.361 Y-11.650; draw !!Xleft+87.361 Ybottom+8.350
G01 F2100.0 X26.632 Y-12.974; draw !!Xleft+86.632 Ybottom+7.026
G01 F2100.0 X25.695 Y-14.015; draw !!Xleft+85.695 Ybottom+5.985
G01 F2100.0 X25.155 Y-14.423; draw !!Xleft+85.155 Ybottom+5.577
G01 F2100.0 X24.568 Y-14.758; draw !!Xleft+84.568 Ybottom+5.242
G01 F2100.0 X23.252 Y-15.204; draw !!Xleft+83.252 Ybottom+4.796
G01 F2100.0 X21.746 Y-15.353; draw !!Xleft+81.746 Ybottom+4.647
G01 F2100.0 X20.866 Y-15.295; draw !!Xleft+80.866 Ybottom+4.705
G01 F2100.0 X20.025 Y-15.123; draw !!Xleft+80.025 Ybottom+4.877
G01 F2100.0 X19.224 Y-14.835; draw !!Xleft+79.224 Ybottom+5.165
G01 F2100.0 X18.462 Y-14.433; draw !!Xleft+78.462 Ybottom+5.567
G01 F2100.0 X17.721 Y-13.900; draw !!Xleft+77.721 Ybottom+6.100
G01 F2100.0 X17.001 Y-13.239; draw !!Xleft+77.001 Ybottom+6.761
G01 F2100.0 X16.301 Y-12.451; draw !!Xleft+76.301 Ybottom+7.549
G01 F2100.0 X15.623 Y-11.535; draw !!Xleft+75.623 Ybottom+8.465
G01 F2100.0 X15.623 Y-14.773; draw !!Xleft+75.623 Ybottom+5.227
G01 F2100.0 X9.714 Y-14.773; draw !!Xleft+69.714 Ybottom+5.227
G01 F2100.0 X9.714 Y7.613; draw !!Xleft+69.714 Ybottom+27.613
G01 F2100.0 X15.623 Y7.613; draw !!Xleft+75.623 Ybottom+27.613
G01 F2100.0 X15.623 Y-3.440; draw !!Xleft+75.623 Ybottom+16.560
G01 F2100.0 X15.689 Y-4.885; draw !!Xleft+75.689 Ybottom+15.115
G01 F2100.0 X15.887 Y-6.163; draw !!Xleft+75.887 Ybottom+13.837
G01 F2100.0 X16.217 Y-7.274; draw !!Xleft+76.217 Ybottom+12.726
G01 F2100.0 X16.679 Y-8.217; draw !!Xleft+76.679 Ybottom+11.783
G01 F2100.0 X17.257 Y-8.969; draw !!Xleft+77.257 Ybottom+11.031
G01 F2100.0 X17.933 Y-9.506; draw !!Xleft+77.933 Ybottom+10.494
G01 F2100.0 X18.709 Y-9.828; draw !!Xleft+78.709 Ybottom+10.172
G01 F2100.0 X19.584 Y-9.936; draw !!Xleft+79.584 Ybottom+10.064
G01 F2100.0 X20.302 Y-9.841; draw !!Xleft+80.302 Ybottom+10.159
G01 F2100.0 X20.937 Y-9.556; draw !!Xleft+80.937 Ybottom+10.444
G01 F2100.0 X21.461 Y-9.096; draw !!Xleft+81.461 Ybottom+10.904
G01 F2100.0 X21.878 Y-8.477; draw !!Xleft+81.878 Ybottom+11.523
G01 F2100.0 X22.117 Y-7.777; draw !!Xleft+82.117 Ybottom+12.223
G01 F2100.0 X22.274 Y-6.758; draw !!Xleft+82.274 Ybottom+13.242
G01 F2100.0 X22.390 Y-2.860; draw !!Xleft+82.390 Ybottom+17.140
G01 F2100.0 X22.357 Y7.613; draw !!Xleft+82.357 Ybottom+27.613
G01 F2100.0 X28.299 Y7.613; draw !!Xleft+88.299 Ybottom+27.613
G01 F2100.0 X28.299 Y-6.058; draw !!Xleft+88.299 Ybottom+13.942
G00 F300.0 Z14.000; pen up !!Zup
G00 F2400.0 X-0.668 Y-3.320; move !!Xleft+59.332 Ybottom+16.680
G00 F300.0 Z10.000; pen down !!Zwork
G01 F2100.0 X6.000 Y7.613; draw !!Xleft+66.000 Ybottom+27.613
G01 F2100.0 X-0.256 Y7.613; draw !!Xleft+59.744 Ybottom+27.613
G01 F2100.0 X-4.035 Y0.977; draw !!Xleft+55.965 Ybottom+20.977
G01 F2100.0 X-7.864 Y7.613; draw !!Xleft+52.136 Ybottom+27.613
G01 F2100.0 X-14.120 Y7.613; draw !!Xleft+45.880 Ybottom+27.613
G01 F2100.0 X-7.452 Y-3.280; draw !!Xleft+52.548 Ybottom+16.720
G01 F2100.0 X-14.450 Y-14.773; draw !!Xleft+45.550 Ybottom+5.227
G01 F2100.0 X-8.195 Y-14.773; draw !!Xleft+51.805 Ybottom+5.227
G01 F2100.0 X-4.035 Y-7.697; draw !!Xleft+55.965 Ybottom+12.303
G01 F2100.0 X0.075 Y-14.773; draw !!Xleft+60.075 Ybottom+5.227
G01 F2100.0 X6.330 Y-14.773; draw !!Xleft+66.330 Ybottom+5.227
G01 F2100.0 X-0.668 Y-3.320; draw !!Xleft+59.332 Ybottom+16.680
G00 F300.0 Z14.000; pen up !!Zup
G00 F2400.0 X-24.650 Y-3.800; move !!Xleft+35.350 Ybottom+16.200
G00 F300.0 Z10.000; pen down !!Zwork
G01 F2100.0 X-28.034 Y8.073; draw !!Xleft+31.966 Ybottom+28.073
G01 F2100.0 X-31.401 Y-3.800; draw !!Xleft+28.599 Ybottom+16.200
G01 F2100.0 X-24.650 Y-3.800; draw !!Xleft+35.350 Ybottom+16.200
G00 F300.0 Z14.000; pen up !!Zup
G00 F2400.0 X-33.002 Y-9.336; move !!Xleft+26.998 Ybottom+10.664
G00 F300.0 Z10.000; pen down !!Zwork
G01 F2100.0 X-34.554 Y-14.773; draw !!Xleft+25.446 Ybottom+5.227
G01 F2100.0 X-40.941 Y-14.773; draw !!Xleft+19.059 Ybottom+5.227
G01 F2100.0 X-31.814 Y15.069; draw !!Xleft+28.186 Ybottom+35.069
G01 F2100.0 X-24.238 Y15.069; draw !!Xleft+35.762 Ybottom+35.069
G01 F2100.0 X-15.110 Y-14.773; draw !!Xleft+44.890 Ybottom+5.227
G01 F2100.0 X-21.498 Y-14.773; draw !!Xleft+38.502 Ybottom+5.227
G01 F2100.0 X-23.066 Y-9.336; draw !!Xleft+36.934 Ybottom+10.664
G01 F2100.0 X-33.002 Y-9.336; draw !!Xleft+26.998 Ybottom+10.664
G00 F300.0 Z14.000; pen up !!Zup
G00 F2400.0 X-44.193 Y15.069; move !!Xleft+15.807 Ybottom+35.069
G00 F300.0 Z10.000; pen down !!Zwork
G01 F2100.0 X-50.547 Y15.069; draw !!Xleft+9.453 Ybottom+35.069
G01 F2100.0 X-50.547 Y-14.773; draw !!Xleft+9.453 Ybottom+5.227
G01 F2100.0 X-44.193 Y-14.773; draw !!Xleft+15.807 Ybottom+5.227
G01 F2100.0 X-44.193 Y15.069; draw !!Xleft+15.807 Ybottom+35.069
G00 F300.0 Z30.000; pen park !!Zpark
