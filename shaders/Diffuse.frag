#version 330

// The camera's position in world-space
uniform vec3 u_cam_pos;

// Color
uniform vec4 u_color;

// Properties of the single point light
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// We also get the uniform texture we want to use.
uniform sampler2D u_texture_1;

// These are the inputs which are the outputs of the vertex shader.
in vec4 v_position;
in vec4 v_normal;

// This is where the final pixel color is output.
out vec4 out_color;

void main() {
  // Diffuse shading: L_d = k_d * (I / r^2) * max(0, n . l)
  vec3 n = normalize(v_normal.xyz);
  vec3 l = normalize(u_light_pos - v_position.xyz);
  float r = length(u_light_pos - v_position.xyz);

  vec3 L_d = u_color.rgb * (u_light_intensity / (r * r)) * max(0.0, dot(n, l));

  out_color = vec4(L_d, 1.0);
}
