#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  return texture(u_texture_2, uv).r;
}

void main() {
  // Build TBN matrix
  vec3 n = normalize(v_normal.xyz);
  vec3 t = normalize(v_tangent.xyz);
  vec3 b = cross(n, t);
  mat3 TBN = mat3(t, b, n);

  // Compute height gradients
  float w = u_texture_2_size.x;
  float ht = u_texture_2_size.y;

  float dU = (h(v_uv + vec2(1.0 / w, 0.0)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  float dV = (h(v_uv + vec2(0.0, 1.0 / ht)) - h(v_uv)) * u_height_scaling * u_normal_scaling;

  // Local displaced normal
  vec3 n_o = vec3(-dU, -dV, 1.0);
  vec3 n_d = normalize(TBN * n_o);

  // Blinn-Phong with displaced normal
  vec3 l = normalize(u_light_pos - v_position.xyz);
  vec3 v = normalize(u_cam_pos - v_position.xyz);
  vec3 half_vec = normalize(l + v);
  float r = length(u_light_pos - v_position.xyz);

  float k_a = 0.1;
  vec3 I_a = vec3(1.0);
  vec3 k_d = u_color.rgb;
  float k_s = 0.5;
  float p = 100.0;

  vec3 ambient = k_a * I_a;
  vec3 diffuse = k_d * (u_light_intensity / (r * r)) * max(0.0, dot(n_d, l));
  vec3 specular = k_s * (u_light_intensity / (r * r)) * pow(max(0.0, dot(n_d, half_vec)), p);

  out_color = vec4(ambient + diffuse + specular, 1.0);
}
