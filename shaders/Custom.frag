#version 330

// (Every uniform is available here.)

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform float u_normal_scaling;
uniform float u_height_scaling;

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// Feel free to add your own textures. If you need more than 4,
// you will need to modify the skeleton.
uniform sampler2D u_texture_1;
uniform sampler2D u_texture_2;
uniform sampler2D u_texture_3;
uniform sampler2D u_texture_4;

// Environment map! Take a look at GLSL documentation to see how to
// sample from this.
uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // Iridescent Fresnel shader with environment reflection
  vec3 n = normalize(v_normal.xyz);
  vec3 viewDir = normalize(u_cam_pos - v_position.xyz);

  // Fresnel effect - stronger color shift at grazing angles
  float fresnel = pow(1.0 - max(0.0, dot(n, viewDir)), 3.0);

  // Iridescent color based on view angle and surface position
  float phase = fresnel * 6.28318 + dot(v_position.xyz, vec3(5.0));
  vec3 iridescentColor = 0.5 + 0.5 * cos(phase + vec3(0.0, 2.094, 4.189));

  // Environment reflection
  vec3 w_o = normalize(v_position.xyz - u_cam_pos);
  vec3 w_i = reflect(w_o, n);
  vec3 envColor = texture(u_texture_cubemap, w_i).rgb;

  // Blend iridescent color with environment based on Fresnel
  vec3 baseColor = mix(iridescentColor, envColor, fresnel);

  // Add Blinn-Phong specular highlight
  vec3 l = normalize(u_light_pos - v_position.xyz);
  vec3 h = normalize(l + viewDir);
  float r = length(u_light_pos - v_position.xyz);
  float spec = pow(max(0.0, dot(n, h)), 64.0);
  vec3 specular = vec3(1.0) * spec * u_light_intensity / (r * r);

  // Subtle rim lighting
  float rim = pow(1.0 - max(0.0, dot(n, viewDir)), 2.0);
  vec3 rimColor = vec3(0.3, 0.5, 1.0) * rim * 0.5;

  out_color = vec4(baseColor + specular + rimColor, 1.0);
}
