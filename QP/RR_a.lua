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
      r = {length.l0/2, 0., 0.},        
      },
    },

  {
      name = "R1_b",
      parent = "R1_a",
      body = bodies.a1,
      joint = joints.rotational_z,
      joint_frame = {
        r = {length.la1, 0., 0.},
                
      },
    },

{
      name = "R1_c",
      parent = "R1_b",
      body = bodies.b1,
      joint = joints.rotational_z,
      joint_frame = {
      r = {length.la1, 0., 0.},        
      },
    },


  },
}
return model
