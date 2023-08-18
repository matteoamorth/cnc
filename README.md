# CNC

CNC with tool radius compensation from repo <https://github.com/pbosetti/c-cnc23>.  
This document provides a description of the code changes.

## Block object

### Lifecycle edits

To enable tool radius compensation, it is necessary to edit the `block` object with a specific field `trc` and a few functions.  
In order to keep track of the original initial point of the block, we need also a new variable `initial_point` inside the block object. This variable is necessary because when we update the previous block value for trc evaluation, we automatically have that the current block has a new initial point that changes the equation of the line/arc of the current block; this causes an error in the equations calculation (see later).  

Added `trc` and `initial_point` fields with getter functions:

```C
data_t trc;
point_t *initial_point;
```

```C
#define block_getter(typ, par, name)    \
  typ block_##name(block_t const *b) {  \
    assert(b);                          \
    return b->par;                      \
  }

block_getter(data_t,trc,trc);
block_getter(point_t *, initial_point, initial_point);
```

> **Note**: every call to `start_point()` function has been replaced with `block_initial_point()` function (see the exceptions later).

In `block_new()` function, `trc` is inherited from previous block `prev` and `initial_point` is set as a new point:

```C
b->trc = prev ? prev->trc : 0;
b->initial_point = point_new();
```

Added function `block_trc_evaluation` to set the `trc` field in `block_set_fields` function:

```C
static block_type_t block_trc_evaluation(block_t *b, char *arg){
  b->trc = (atoi(arg) == 41) ? (-1) : ((atoi(arg) == 42) ? 1 : 0);
  return (block_type_t) atoi(arg);
}
```

```C
static int block_set_fields(block_t *b, char cmd, char *arg) {
  assert(b && arg);
  switch (cmd) {
    case 'G':
    b->type = block_trc_evaluation(b, arg);
    break;
    ...
  }
```

### Mathematical problem definition - line equation


Given the equation of a generic line:

$$ y = ax + b$$

It is possible to evaluate the parameters $a$ and $b$ with these two equations:

$$ a = {y_2 - y_1 \over x_2 - x_1}$$

$$ b = {x_2 y_1 - x_1 y_2 \over x_2 - x_1} $$

where initial point is $P_1(x_1, y_1)$ and final point is $P_2(x_2, y_2)$

It is important to remember special cases:

- horizontal ($y_1 = y_2$):

  $$y = y_1$$ 

- vertical ($x_1 = x_2$):

  $$x = x_1$$

> **Note**: the vertical case is a strange exception that must be considered. The function `block_equation` can return a special value (for example `bool`), that indicates whether or not it is a vertical line.

### Line equation code

The `block_equation()` calculates the equation of a line between two points. The `a` and `b` pointers are the line equation parameters. If the return value is `true`, the equation is $x = a$.

```c
static bool block_equation(data_t *a, data_t *b, point_t const *p_init, point_t const *p_final){
  data_t temp_a, temp_b;
  bool vertical = false;
  if(point_x(p_init) == point_x(p_final)){
    vertical = true;
    temp_a = point_x(p_init);
    temp_b = 0;

  } else { 
    //other cases
    point_t *p_dist = point_new();
    point_delta(p_init, p_final, p_dist);

    temp_a = point_y(p_dist) / point_x(p_dist);
    temp_b = (point_x(p_final) * point_y(p_init) - point_x(p_init) * point_y(p_final)) / point_x(p_dist);
  }

  memcpy(a, &temp_a, sizeof(data_t));
  memcpy(b, &temp_b, sizeof(data_t));
  return vertical;
}
```

### Offset sign of the line equation

Now the equation of line with offset can be evaluated:

$$ y = ax + b ± \sqrt{a^2 +1}$$

The sign before the square root can be defined through these points:

- if we have the **left case** (`G41`), the sign will be **plus** when we move from left to right and from bottom to top.
- if we have the **right case** (`G42`), the sign will be **minus** when we move from left to right and from bottom to top.

> **Note**: we assume that the `x` axis increases from left to right and the `y` axis increases from bottom to top.

Considering that the **direction** its known, thanks to the $a$ parameter of the line's equation, we **get the sign** just multiplying $a$ sign* with the offset type imposed by program (`G41` or `G42`).

The function `block_eq_sign` is designed to provide the sign

```c
static int block_eq_sign(point_t *from, point_t *to, data_t const trc){

  // if trc = 1  => right 
  // if trc = 0  => no
  // if trc = -1 => left
  point_t *p_dist = point_new();
  point_delta(from, to, p_dist);

  if((point_x(p_dist) > 0) && (point_y(p_dist) >= 0)) return -trc;
  if((point_x(p_dist) <= 0) && (point_y(p_dist) >= 0)) return trc;
  if((point_x(p_dist) >= 0) && (point_y(p_dist) <  0)) return -trc;
  if((point_x(p_dist) < 0) && (point_y(p_dist) < 0)) return trc;

  return 0;

}
  ```

> **Note**: a few comparison conditions have the equal sign because vertical and horizontal cases must be considered.

### Mathematical problem definition - lines intersection

Given two line equations:

$$ y = a_1x + b_1$$
$$ y = a_2x + b_2$$

The generic intersection point coordinates calculated follow these formulas:

$$ x = {b_2 - b_1 \over a_1 - a_2}$$
$$ y = {b_1  a_2 - b_2  a_1 \over a_2 - a_1} $$

### Lines intersection code

The `intersection_line_line()` function evaluates the intersection point, considering also the special cases of vertical and collinear lines.  

In this section, the first block is not vertical while the second is vertical.

```c
if(!vertical1 && vertical2){
    x = point_x(block_target(b_line2)) + sign_trc1 * tool_radius;
    // y = m * x + q
    y = a1 * x + b1;
    point_set_xyz(p_new_target, x, y, point_z(block_target(b_line1)));
    return p_new_target;
  }
```

### Mathematical problem definition - arc equation

To describe an arc it is possible to use the generic equation of a circle:

$$ (x - X_c)^2 + (y - Y_c)^2 = r^2 $$

*where $X_c$ and $Y_c$ are circle's center coordinates.*

### Offset sign of the circle equation

In the circle block the program must evaluate if the radius must be increased or not. This table sums up the logic considered.

|TRC             |    CW         |   CCW        |
|----------------|---------------|--------------|
|RIGHT TRC (+1)  | $r - tool_r$  | $r - tool_r$ |
|LEFT  TRC (-1)  | $r + tool_r$  | $r + tool_r$ |

The operation can be performed with this line:

```c
temp_radius = (b_arc->type == ARC_CCW) ? (b_arc->trc + b_arc->trc * tool_radius) : (b_arc->trc - b_arc->trc * tool_radius);
```

### Mathematical problem definition - arc and line intersection 

Given the generic equation of line and the generic equation of circle, it is possible to calculate two different points of intersection:

$$y = a x + b$$
$$ (x - X_c)^2 + (y - Y_c)^2 = r^2 $$


$$ (a^2 + 1)x^2 + 2(a  b - X_c - a  Y_c) x + X_c^2 + b^2 + Y_c^2 -2  b  Y_c - r^2 = 0 $$

> **Note**: when the program evaluates the trc of blocks, one of the two points found is the solution, the other one must be discarted. To choose the correct point, a comparison between the original intersection point and the two solution is performed. The closest is the correct point.

### Arc and line intersection code

The `intersection_arc_line()` function calculates the intersection point, considering also the special case of vertical lines.  

```c
point_t *intersection_arc_line(block_t *b_arc, block_t *b_line, point_t const *target, bool change_radius)
```

Where `b_arc` is the arc block, `b_line` is the line block, `target` the original target without trc and `change_radius` indicates wheter or not to change the arc radius. 
> The radius is changed only when the first block is an arc and the second is a line.

> The collinear case (line tangential to circle) provides a solution point twice.

## Machine object

In this section the `machine` object and `machine.ini` configuration have been modified to allow the tools upload to the system.

### Configuration file - machine.ini

Added `tools` list and tools number `tool_n` fields:

```toml
tools = [10.0, 2.5, 3.0, 4.0, 7.5]
tools_n = 5
```

### Machine object - tools upload

Defined max number of tools allowed by machine `MAX_TOOLS` and new `machine` object fields `tools_n` and `tools`:

```c
#define MAX_TOOLS 5
data_t tools[MAX_TOOLS];
data_t tools_n;
```

> The `tools` array contains the tools radii while `tools_n` the number of tools avaiable

Upload tools radii in machine `m`:

```c
//extract tools, if missing close program 
tools_list = toml_array_in(ccnc, "tools");
if(!tools_list){  
  eprintf("Could not find any tool!\n");
  goto fail;
}

if(m->tools_n > MAX_TOOLS){
  wprintf("Tools provided are more than the maximum allowed, 
           the machine loads only the first %d elements", MAX_TOOLS);
  m->tools_n = MAX_TOOLS;
}

for (size_t i = 0; i < m->tools_n; i++)
  m->tools[i] = toml_double_at(tools_list,i).u.d;

```

> **Note**: if the number of tools provided exceeds the maximum allowed, the system uploads only the first `MAX_TOOLS` elements.

Add function `machine_tool_radius` to get radius of a given tool number:

```c
data_t machine_tool_radius(machine_t *m, data_t i){
  return m->tools[(int)i];
}
```

## Further modifications on block object

Finally, in order to inherit the `target_point` of the previous block, we add in function `block_parse` these lines (before `trc` of the previous block):

```c
// inherit coordinates from previous point
  p0 = start_point(b);

  point_set_x(b->initial_point, point_x(p0));
  point_set_y(b->initial_point, point_x(p0));
  point_set_z(b->initial_point, point_z(p0));

```


## Offset evaluation

The sequence of blocks may change. We consider these different cases

### Case line -> line

Evaluate the two equations of line:

```c
bool vertical = false;
bool vertical1 = false;
data_t a1, a2, b1, b2, x, y;
point_t *p_new_target = point_new();

vertical = block_equation(&a1, &b1, block_initial_point(b->prev), block_target(b->prev));
vertical1 = block_equation(&a2, &b2, block_initial_point(b), block_target(b));
int sign_trc = block_eq_sign(block_initial_point(b->prev),block_target(b->prev), b->trc);
int sign_trc1 = block_eq_sign(block_initial_point(b),block_target(b), b->trc);
```

As previously explained, the presence of vertical lines must be considered plus the collinear lines. In order we have: 
- both lines are not vertical 
- one line is vertical
- both lines vertical

```c
if(!vertical && vertical1){
          x = point_x(block_target(b)) + sign_trc1 * tool_radius;
          // y = m * x + q
          y = a1 * x + b1;
        }

if(vertical && !vertical1){
  x = point_x(block_target(b->prev)) + sign_trc1 * tool_radius;
  // y = m * x + q
  y = a2 * x + b2;
}

// both vertical
if(vertical && vertical1){
  x = point_x(block_target(b->prev)) + sign_trc * tool_radius;
  y = point_x(block_target(b->prev));
}

// evaluate previous block - > must edit also the starting point 
        // because it is not inherited when it is modified in trc evaluation
p0 = block_initial_point(b->prev);
point_modal(p0, b->prev->target);

//new target point of previous block 
point_set_x(block_target(b->prev), x);
point_set_y(block_target(b->prev), y);

//evaluation
point_delta(p0, b->prev->target, b->prev->delta);
b->prev->length = point_dist(p0, b->prev->target);
block_compute(b->prev);
```

### Case arc -> line

First we evaluate arc, than we recalculate parameters after trc

```c
if (block_arc(b->prev)) {
  wprintf("Could not calculate arc coordinates\n");
  rv++;
  break;
}

b->prev->acc = machine_A(b->prev->machine) / 2.0;
b->prev->arc_feedrate = MIN( b->prev->feedrate,
                        pow(3.0 / 4.0 * pow(machine_A(b->prev->machine), 2) * pow(b->prev->r, 2), 0.25) * 60);

// trc reduces or increses the radius of the arc

// sign(r+-r_tool) |    CW       |   CCW
// --------------------------------------------
// RIGHT TRC (+1)  | r - r_tool  | r - r_tool
// LEFT  TRC (+1)  | r + r_tool  | r + r_tool  

// change radius
if (b->prev->type == ARC_CCW)
  b->prev->r += b->prev->trc * tool_radius;

if (b->prev->type == ARC_CW)
  b->prev->r -= b->prev->trc * tool_radius;

vertical1 = block_equation(&a2, &b2, block_initial_point(b), block_target(b));
sign_trc1 = block_eq_sign(block_initial_point(b),block_target(b), b->trc);
b2 += sign_trc1 * tool_radius * sqrt(a2 * a2 + 1.0);
point_t *center_point = block_center(b->prev);
point_t *p1 = point_new();
point_t *p2 = point_new();
data_t x1, x2, y1, y2;

if(vertical1){
  x = point_x(block_target(b)) + sign_trc1 * tool_radius;
  
  point_set_x(p1, x);
  point_set_x(p2, x);

  // y = yc +- sqrt(r - (x-xc)^2)
  y1 = point_y(center_point) + sqrt(block_r(b->prev) - pow(x - point_x(center_point),2));
  y2 = point_y(center_point) - sqrt(block_r(b->prev) - pow(x - point_x(center_point),2));

  point_set_y(p1, y1);
  point_set_y(p2, y2);
  
  //set closest point as new target 
  p_new_target = point_dist(b->prev->target, p1) < point_dist(b->prev->target, p2) ? p1 : p2;

} else { // generic case 

/* mathematical computation

After some iterations the equation of the intersection is the following:
(a2^2 + 1)x^2 + 2(a2 * b2 - Xc - a2 * Yc)x + Xc^2 + b2^2 + Yc^2 -2 * b2 * Yc - r^2 = 0

With the formula x = (-b +- sqrt(b^2 - 4 * a * c))/ (2 * a) there are two possible solutions
*/

//(a2^2 + 1)
data_t A = pow(a2, 2) + 1;

//2(a2 * b2 - Xc - a2 * Yc)
data_t B = 2 * (a2 * (b2 - point_y(center_point)) - point_x(center_point));

// +Xc^2 + b2^2 + Yc^2 -2 * b2 * Yc - r^2
data_t C = + pow(point_x(center_point), 2) + pow(b2, 2) + pow(point_y(center_point), 2) - 2 * b2 * point_y(center_point) - pow(tool_radius, 2);

x1 = (-B + sqrt(pow(B,2) - 4 * A * C)) / 2 * A;
x2 = (-B - sqrt(pow(B,2) - 4 * A * C)) / 2 * A;

point_set_x(p1, x);
point_set_x(p2, x);

// use eq of line to find y
y1 = a2 * x1 + b2;
y2 = a2 * x2 + b2;

point_set_y(p1, y1);
point_set_y(p2, y2);

//set closest point as new target 
p_new_target = point_dist(b->prev->target, p1) < point_dist(b->prev->target, p2) ? p1 : p2;

}

p0 = block_initial_point(b->prev);
point_modal(p0, b->prev->target);

//new target point of the arc block 
point_set_x(block_target(b->prev), point_x(p_new_target));
point_set_y(block_target(b->prev), point_y(p_new_target));

//inherit starting point of arc from previous block 

block_compute(b->prev);


b->prev->arc_feedrate = MIN( b->prev->feedrate, pow(3.0 / 4.0 * pow(machine_A(b->prev->machine), 2) * pow(b->prev->r, 2), 0.25) * 60);

//at the end of the arc case
if (block_arc(b)) {
wprintf("Could not calculate arc coordinates\n");
rv++;
return rv;
}
```

