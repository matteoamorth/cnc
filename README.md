# CNC
CNC with tool radius compensation from repo https://github.com/pbosetti/c-cnc23

## Add support to G-CODE G40/G41/G42 command

Add field `trc` in block object:

```C
data_t trc;
```

Added function `block_trc_evaluation` to set the `trc` field in `block_set_fields` function

(to be checked with `block_parse` modification function later):
```C
//trc evaluation 
static block_type_t block_trc_evaluation(block_t *b, char *arg){
  block_type_t i = (block_type_t) atoi(arg);
  switch (i){
  case 41:
    b->trc = -1; break;
  case 42:
    b->trc = 1; break;
  default:
  }
  return i;
}
```


Add a getter function for `trc` value (Added a line in the macro):


```C
#define block_getter(typ, par, name)    \
  typ block_##name(block_t const *b) {  \
    assert(b);                          \
    return b->par;                      \
  }

block_getter(data_t,trc,trc);
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
bool block_equation(data_t *a, data_t *b, point p_init, point p_final){
    bool vertical = false;
    if(p_init->x == p_final->x){
      vertical = true
      b = p_init->x; 
      a = 0;
      return vertical;
    }
    
//other cases
    point *p_dist = point_new();
    point_delta(p_init, p_final, p_dist);

    a = p_dist-> y / p_dist->x;

    b = (p_final->x * p_init->y - p_init->x * p_final->y) / p_dist->x;

    return vertical;
}
```

### Line equation through two points
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

*\* we must consider the special case with vertical line* $a = 0$ .

  ```c
  ```

