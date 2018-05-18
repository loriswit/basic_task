#include "../util/camera.h"

#include "color.h"

#define BLUE_H_MIN  0.5
#define BLUE_H_MAX  0.8
#define BLUE_S_MIN  0.3
#define BLUE_S_MAX  1
#define BLUE_L_MIN  0.1
#define BLUE_L_MAX  0.75

#define RED_H_MIN   0
#define RED_H_MAX   0.2
#define RED_S_MIN   0.2
#define RED_S_MAX   1
#define RED_L_MIN   0.1
#define RED_L_MAX   0.75

#define get_max(a, b) (a) > (b) ? (a) : (b)
#define get_min(a, b) (a) < (b) ? (a) : (b)

#define is_between(value, min, max) ((value) >= (min) && (value) <= (max))

#define is_red(color) is_between((color).h, RED_H_MIN, RED_H_MAX) \
                   && is_between((color).s, RED_S_MIN, RED_S_MAX) \
                   && is_between((color).l, RED_L_MIN, RED_L_MAX)

#define is_blue(color) is_between((color).h, BLUE_H_MIN, BLUE_H_MAX) \
                    && is_between((color).s, BLUE_S_MIN, BLUE_S_MAX) \
                    && is_between((color).l, BLUE_L_MIN, BLUE_L_MAX)

typedef struct
{
    float h;
    float s;
    float l;
} hsl_color;

hsl_color to_hsl(rgb_color input)
{
    unsigned max = get_max(get_max(input.r, input.g), input.b);
    unsigned min = get_min(get_min(input.r, input.g), input.b);
    
    float maxf = (float) max / 255;
    float minf = (float) min / 255;
    
    float hue = 0;
    float sat = 0;
    float light = (maxf + minf) / 2;
    
    if(max != min)
    {
        float rf = (float) input.r / 255;
        float gf = (float) input.g / 255;
        float bf = (float) input.b / 255;
        float delta = maxf - minf;
        
        if(light > 0.5)
            sat = delta / (2 - maxf - minf);
        else
            sat = delta / (maxf + minf);
        
        if(max == input.r)
            hue = (gf - bf) / delta + (gf < bf ? 6 : 0);
        else if(max == input.g)
            hue = (bf - rf) / delta + 2;
        else if(max == input.b)
            hue = (rf - gf) / delta + 4;
        hue /= 6;
    }
    
    return (hsl_color) {hue, sat, light};
}

color_t get_color()
{
    rgb_color average = camera_get_average_color();
    hsl_color color = to_hsl(average);
    
    if(is_blue(color))
        return COLOR_BLUE;
    
    else if(is_red(color))
        return COLOR_RED;
    
    else
        return COLOR_NONE;
}

bool color_matches(color_t a, color_t b)
{
    if(a == COLOR_NONE || b == COLOR_NONE)
        return false;
    if(a == COLOR_ANY)
        return b != COLOR_NONE;
    if(b == COLOR_ANY)
        return a != COLOR_NONE;
    else
        return a == b;
}
