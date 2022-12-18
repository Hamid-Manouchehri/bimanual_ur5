function diag (xx, yy, zz) 
	return {
		{xx, 0.0, 0.0},
		{0.0, yy, 0.0},
		{0.0, 0.0, zz}
	}
end

function MoI (m, d, w) 
	return 
		1/12*m*(d*d + w*w)
	
end

PI = 3.141592653589793

grav = {0., -9.81*0, 0.}


length = {
la1 = 0.4, lb1 = 0.15, la2 = 0.4, lb2 = 0.15, lo = .25, l0 = .5, w = .1, wo = .15
}

masses = {
ma1 = 1.5, mb1 = 1., ma2 = 1.5, mb2 = 1., mo = 2.
}

inertias = {
ia1_xx = MoI (masses.ma1, length.w, length.w), ia1_yy = MoI (masses.ma1, length.la1, length.w), ia1_zz = MoI (masses.ma1, length.la1, length.w), 
ib1_xx = MoI (masses.ma1, length.w, length.w), ib1_yy = MoI (masses.ma1, length.lb1, length.w), ib1_zz = MoI (masses.ma1, length.lb1, length.w), 
ia2_xx = MoI (masses.ma1, length.w, length.w), ia2_yy = MoI (masses.ma1, length.la2, length.w), ia2_zz = MoI (masses.ma1, length.la2, length.w), 
ib2_xx = MoI (masses.ma1, length.w, length.w), ib2_yy = MoI (masses.ma1, length.lb2, length.w), ib2_zz = MoI (masses.ma1, length.lb2, length.w), 
io_xx = MoI (masses.ma1, length.wo, length.wo), io_yy = MoI (masses.ma1, length.lo, length.wo), io_zz = MoI (masses.ma1, length.lo, length.wo)
}


parta1 = { mass = masses.ma1, com = { 1/2*length.la1, 0., 0.}, inertia = diag (inertias.ia1_xx,  inertias.ia1_yy, inertias.ia1_zz) }
partb1 = { mass = masses.mb1, com = { 1/2*length.lb1, 0., 0.}, inertia = diag (inertias.ib1_xx,  inertias.ib1_yy, inertias.ib1_zz) }
parta2 = { mass = masses.ma2, com = { 1/2*length.la2, 0., 0.}, inertia = diag (inertias.ia2_xx,  inertias.ia2_yy, inertias.ia2_zz) }
partb2 = { mass = masses.mb2, com = { 1/2*length.lb2, 0., 0.}, inertia = diag (inertias.ib2_xx,  inertias.ib2_yy, inertias.ib2_zz) }
parto = { mass = masses.mo, com = { 1/2*length.lo, 0., 0.}, inertia = diag (inertias.io_xx,  inertias.io_yy, inertias.io_zz) }

bodies = {
  a1 = parta1,
  b1 = partb1,
  a2 = parta2,
  b2 = partb2,
  object = parto,
}

joints = {
  freeflyer2d = {
    { 0., 0., 0., 1., 0., 0.},
    { 0., 0., 0., 0., 1., 0.},
    { 0., 0., 1., 0., 0., 0.}
  },
  rotational_z = {
    { 0., 0., 1., 0., 0., 0.}
  },
  fixed = {}
}
