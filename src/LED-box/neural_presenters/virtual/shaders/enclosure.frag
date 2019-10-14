#version 120

#define MAX_LEDS 300
#define ENCLOSURE_SPREAD_MAX 30000
#define ENCLOSURE_SPREAD_MIN 8000
#define LED_AMPLIFICATION 2
#define ENCLOSURE_FROSTING 30.0
#define FALLOFF 0.1
#define ENCLOSURE_COLOR vec3(1.0, 1.0, 1.0)

varying vec4 world_normal;
varying vec4 world_position;
varying vec3 view_vector;

uniform vec4[MAX_LEDS] led_positions;
uniform vec4[MAX_LEDS] led_colors;
uniform vec2 hdr;

void main() {

    vec3 ambient_light = vec3(1.0, 1.0, 1.0);
    vec3 ambient_normal = vec3(0.474, 0.316, 0.158);
    float ambient_bounce = 0.8;

    float intensity = clamp(dot(world_normal.xyz, ambient_normal), 0, 1);
    intensity = intensity * (1 - ambient_bounce) + ambient_bounce;
    vec3 color_sum = ENCLOSURE_COLOR * ambient_light * intensity * 0.5;
    //float change_sum = 0.0;

    for (int i = 0; i < led_colors.length(); i++) {

        vec3 distance_vector = world_position.xyz - led_positions[i].xyz;

        vec3 projection_plane_distance = dot(view_vector, distance_vector) * view_vector;
        vec3 projected_distance_vector = projection_plane_distance - distance_vector;
        //vec3 adjustment_vector = (dot(view_vector, polygon_distance_vector) * view_vector) - polygon_distance_vector;

        //float normal_distance_squared = dot(world_normal, distance_vector); // Using just normal here yields '3D'
        //float planar_distance_squared = distance_squared - normal_distance_squared;

        //vec3 corrected_distance_vector = distance_vector - adjustment_vector;
        //float corrected_distance_squared = dot(corrected_distance_vector, corrected_distance_vector);
        float projected_distance_squared = dot(projected_distance_vector, projected_distance_vector);
        if (projected_distance_squared > 0.002) continue;

        //float polygon_distance = sqrt(abs(dot(world_normal.xyz, distance_vector)));
        float distance_falloff = pow(1 - clamp(length(projection_plane_distance)/FALLOFF, 0, 1),2);
        if (distance_falloff == 0) continue;

        float spread = exp(-(distance_falloff*(ENCLOSURE_SPREAD_MAX-ENCLOSURE_SPREAD_MIN) + ENCLOSURE_SPREAD_MIN) * projected_distance_squared / 2.0);
        //change_sum += gaussian_spread;
        //float spread = 1/sqrt(distance_squared);

        vec4 led_color = led_colors[i];
        float bias = mod(dot(world_position,world_position)*52461156.0+25651584.0,1000)/1000.0;
        bias = 1.0 - bias/ENCLOSURE_FROSTING;

        color_sum += led_color.xyz * spread * bias * LED_AMPLIFICATION * distance_falloff;
    }

    color_sum = color_sum / hdr.x - hdr.y;

    //change_sum = clamp(1 - change_sum, 0, 1);
    //color_sum += ENCLOSURE_COLOR * change_sum;

    gl_FragColor = vec4(color_sum, 1.0);
}