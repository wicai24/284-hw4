#version 330

uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // Compute outgoing eye-ray direction
  vec3 w_o = normalize(v_position.xyz - u_cam_pos);

  // Reflect across surface normal
  vec3 w_i = reflect(w_o, normalize(v_normal.xyz));

  // Sample environment cubemap
  out_color = texture(u_texture_cubemap, w_i);
  out_color.a = 1.0;
}
