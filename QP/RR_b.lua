dofile "./Param.lua"

model = {
  gravity = grav,

  frames = {

    {
      name = "R2_a",
      parent = "ROOT",
      body = bodies.a2,
      joint = joints.rotational_z,
      joint_frame = {
      r = {- length.l0/2, 0., 0.},        
      },
    },

  {
      name = "R2_b",
      parent = "R2_a",
      body = bodies.a2,
      joint = joints.rotational_z,
      joint_frame = {
        r = {length.la2, 0., 0.},
                
      },
    },

{
      name = "R2_c",
      parent = "R2_b",
      body = bodies.b2,
      joint = joints.rotational_z,
      joint_frame = {
      r = {length.la2, 0., 0.},        
      },
    },


  },
}
return model
