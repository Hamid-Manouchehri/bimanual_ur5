dofile "./Param.lua"

model = {
  gravity = grav,

  frames = {

    {
      name = "R1_a",
      parent = "ROOT",
      body = bodies.a1,
      joint = joints.rotational_z,
      joint_frame = {
      r = {0., 0., 0.},
      E = {
		{0.0, 0.0, 0.0},
		{0.0, 0, 0.0},
		{0.0, 0.0, 0.0}
	      },        
      },
    },

  {
      name = "R1_b",
      parent = "R1_a",
      body = bodies.b1,
      joint = joints.rotational_z,
      joint_frame = {
        r = {length.la1, 0., 0.},
                
      },
    },


    {
      name = "Obj",
      parent = "R1_2",
      body = bodies.object,
      joint = joints.rotational_z,
      joint_frame = {
        r = {length.lb1, 0., 0.},
      },
    },



    {
      name = "R2_b",
      parent = "Obj",
      body = bodies.b2,
      joint = joints.rotational_z,
      joint_frame = {
        r = {length.lo, 0., 0.}
      },
    },


    {
      name = "R2_a",
      parent = "R2_b",
      body = bodies.a2,
      joint = joints.rotational_z,
      joint_frame = {
        r = {length.lb2, 0., 0.}
      },
    },

 


  },
}
return model
