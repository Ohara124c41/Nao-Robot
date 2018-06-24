'''
psuedo code for basic Monte Carlo

NB: orientation matters only in the resampling stage since 
prediction is different for different orientations

MCL(prev_x_t, u_t, z_t)
x_tbar = x_t = 0
for m=1 to M:
	xm_t = (u_t, prev_x_t)
	wm_t = (z_t, xm_t)
	x_tbar = x_tbar+<xm_t, wm_t>
	endfor
	for m=1 to M:
		draw xm_t from x_tbar with P approx wm_t
		x_t = x_t+ xm_t
	return x_t
```