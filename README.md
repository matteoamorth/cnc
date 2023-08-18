# CNC
CNC with tool radius compensation from repo https://github.com/pbosetti/c-cnc23

This document provides a description of the code changes.

## Add support to G-CODE G40/G41/G42 command

Added field `trc` and `initial_point` in block object:

```C
data_t trc;
point_t *initial_point;
```
Added getter functions for `trc`  and `initial_point` fields:

```C
#define block_getter(typ, par, name)    \
  typ block_##name(block_t const *b) {  \
    assert(b);                          \
    return b->par;                      \
  }

block_getter(data_t,trc,trc);
```

Added function `block_trc_evaluation` to set the `trc` field in `block_set_fields` function:

```C
static block_type_t block_trc_evaluation(block_t *b, char *arg){
  b->trc = (atoi(arg) == 41) ? (-1) : ((atoi(arg) == 42) ? 1 : 0);
  return (block_type_t) atoi(arg);
}
```



Set `trc` in `block_new` function inherited from previous block `prev`:

```C
if(prev)
  b->trc = prev->trc;
else
  b->trc = 0;
```

## Edit machine object
Defined max number of tools allowed by machine `MAX_TOOLS`:

```c
#define MAX_TOOLS 5
```

In order to make the program work, the file `machine.ini` must include `tools` list and tools number `tool_n` like this example:

```toml
tools = [10.0, 2.5, 3.0, 4.0, 7.5]
tools_n = 5
```

Add `tools` list and number of tools avaiable `tools_n` in `machine` object:
```c
data_t tools[MAX_TOOLS];
data_t tools_n;
```

Upload tools in machine `m`:
```c
//extract tools, if missing close program 
tools_list = toml_array_in(ccnc, "tools");
if(!tools_list){  
  eprintf("Could not find any tool!\n");
  goto fail;
}

if(m->tools_n > MAX_TOOLS){
  wprintf("Tools provided are more than the maximum allowed, the machine loads only the first %d elements", MAX_TOOLS);
  m->tools_n = MAX_TOOLS;
}

for (size_t i = 0; i < m->tools_n; i++)
  m->tools[i] = toml_double_at(tools_list,i).u.d;

```

Add function `machine_tool_radius` to get radius of a given tool number:
```c
data_t machine_tool_radius(machine_t *m, data_t i){
  return m->tools[(int)i];
}
```

## Further modifications on block object

In order to keep track of the original initial point of the block, we need to add a new variable `initial_point` inside the block object. This variable is necessary because when we update the previous block value for trc evaluation, we automatically have that the current block has a new initial point that changes the equation of the line/arc of the current block; this cause an error not negligible.

```c
point_t *initial_point;
```

In consequence, every call of `start_point()` function is substituted with the new line in macro getter `block_initial_point()`:

```c
#define block_getter(typ, par, name)       \
  typ block_##name(block_t const *b) {     \
    assert(b);                             \
    return b->par;                         \
  }

block_getter(point_t *, initial_point, initial_point);
```

Finally, in order to inherit the `target_point` of the previous block, we add in function `block_parse` these lines (before `trc` of the previous block):

```c
// inherit coordinates from previous point
  p0 = start_point(b);

  point_set_x(b->initial_point, point_x(p0));
  point_set_y(b->initial_point, point_x(p0));
  point_set_z(b->initial_point, point_z(p0));

```

## Math functions 

### Generic line equation 
Given the equation of a generic line:

$$ y = ax + b$$

It is possible to evaluate the parameters $a$ and $b$ with these two equations:

$$ a = {y_2 - y_1 \over x_2 - x_1}$$

$$ b = {x_2 y_1 - x_1 y_2 \over x_2 - x_1} $$

where initial point is $P_1(x_1, y_1)$ and final point is $P_2(x_2, y_2)$

It is important to remember special cases: 
- horizontal ($y_1 = y_2$).

  $$y = y_1$$ 

- vertical ($x_1 = x_2$) 

  $$x = x_1$$ 

The vertical case is a strange exception that must be considered in the code. 
If the function `block_equation` returns a type (for example `bool`), we can return a special value that suggests we are in the vertical special case.
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

### Line equation through two points (not used)
If we restart from the generic equation defined by two points:

$${y-y_1 \over x-x_1} = {y_2-y_1 \over x_2-x_1}$$

we can obtain this result:

$$(y_1-y_2) * x + (x_2-x_1) * y + (x_1-x_2)*y_1 + (y_2-y_1)*x_1 = 0$$

Now if we assume that:
- $a_0 = y_1-y_2$
- $b_0 = x_2-x_1$
- $c_0 = (x_1-x_2)*y_1 + (y_2-y_1)*x_1$

The equation will be:
$$a_0 * x + b_0 * y + c_0 = 0$$

Now, to evaluate $a$ and $b$ of the equation of line we can simply divide by $-b_0$:
- $a = -{a_0 \over b_0}$
- $b = -{c_0 \over b_0}$

If $b_0 = 0$, the value of $b$ will be:
$$b = -{c_0 \over a_0}$$

In conclusion, we can create a function 

```c
void block_equation(data_t *a, data_t *b, data_t *c, point p_init, point p_final){
    point p_dist = point_new();
    point_delta(p_init, p_final, p_dist);
    a = - p_dist->y;
    b =   p_dist->x;
    c = (- p_dist->x) * p_init->y + (p_dist->y) * p_init->y;
    return;
}
```

Now the equation of line with offset can be evaluated:

$$ y = ax + b ± \sqrt{a^2 +1}$$ 

The sign before the square root can be defined through these points:
 
- if we have the **left case** (`G41`), the sign will be **plus** when we move from left to right and from bottom to top*.
- if we have the **right case** (`G42`), the sign will be **minus** when we move from left to right and from bottom to top*.

*\* we assume that the `x` axis increases from left to right and the `y` axis increases from bottom to top.*

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

