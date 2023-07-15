# CNC
CNC with tool radius compensation from repo https://github.com/pbosetti/c-cnc23

## Add support to G-CODE G40/G41/G42 command

Add field `trc` in block object:

```C
data_t trc;
```

Added function `block_trc_evaluation` to set the `trc` field in `block_set_fields` function

(dummy function, needs modifications)
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


Add a getter function for `trc` value (Added a line in the macro).


```C
#define block_getter(typ, par, name)    \
  typ block_##name(block_t const *b) {  \
    assert(b);                          \
    return b->par;                      \
  }

block_getter(data_t,trc,trc);
```

Set `trc` in `block_new` function inherited from previous block `prev`  

```C
if(prev)
  b->trc = prev->trc;
else
  b->trc = 0;
```
