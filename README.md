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
tools_n = 5;
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

