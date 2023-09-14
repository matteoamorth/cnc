//   ____  _            _
//  | __ )| | ___   ___| | __
//  |  _ \| |/ _ \ / __| |/ /
//  | |_) | | (_) | (__|   <
//  |____/|_|\___/ \___|_|\_\
//
#include "block.h"
#include "defines.h"
#include "machine.h"
#include <ctype.h> // toupper()
#include <math.h>
#include <sys/param.h> // MIN()

#define TRC_FEATURE 1

//   ____            _                 _   _
//  |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
//  | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
//  | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
//  |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//
// Velocity profile data
typedef struct {
  data_t a, d;             // acceleration and deceleration
  data_t f, l;             // nominal feedrate and length
  data_t fs, fe;           // initial and final feedrate
  data_t dt_1, dt_m, dt_2; // trapezoidal profile times
  data_t dt;               // total block duration
} block_profile_t;

// Object struct (opaque)
typedef struct block {
  char *line;               // G-code string
  block_type_t type;        // block type
  size_t n;                 // block number
  size_t tool;              // tool number
  data_t feedrate;          // feedrate in mm/min
  data_t arc_feedrate;      // actual nominal feedrate along an arc motion
  data_t spindle;           // spindle rotational speed in RPM
  point_t *target;          // final coordinate of this block
  point_t *delta;           // projections
  point_t *center;          // arc center coordinates
  data_t length;            // segment of arc length
  data_t i, j, r;           // arc parameters
  data_t theta0, dtheta;    // initial and arc angles
  data_t acc;               // actual acceleration
  data_t trc;               //tool radius compensation
  block_profile_t *prof;    // block velocity profile data
  machine_t const *machine; // machine object (holding config data)
  struct block *prev;
  struct block *next;

  // custom add-on
  point_t *initial_point; //initial target of this block

} block_t;

// STATIC FUNCTIONS
static point_t *start_point(block_t *b);
static int block_set_fields(block_t *b, char cmd, char *argv);
static void block_compute(block_t *b);
static int block_arc(block_t *b);

#if TRC_FEATURE
static void block_trc_evaluation(block_t *b, char *arg);
static bool block_equation(data_t *a, data_t *b, point_t const *p_init, point_t const *p_final);
static int block_eq_sign(point_t *from, point_t *to, data_t const trc);
static point_t *intersection_arc_line(block_t *b_arc, block_t *b_line, point_t const *target, bool change_radius);
static point_t *intersection_line_line(block_t *b_line1, block_t *b_line2);
#endif


//   _____                 _   _
//  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
//  | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
//  |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
//  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
//

// LIFECYCLE ===================================================================
block_t *block_new(char const *line, block_t *prev, machine_t const *machine) {
  assert(line);
  // allocate memory
  block_t *b = malloc(sizeof(block_t));
  if (!b) {
    eprintf("Could not allocate memory for a block\n");
    goto fail;
  }

  if (prev) { // copy the memory from the previous block
    memcpy(b, prev, sizeof(block_t));
    b->prev = prev;
    prev->next = b;
  } else { // this is the first block: set everything to 0
    memset(b, 0, sizeof(block_t));
  }

  // inherit trc from previous block
  b->trc = prev ? prev->trc : 0;

  // in any case all non-modal parameters are set to 0
  b->i = b->j = b->r = 0;
  b->length = 0;
  b->type = NO_MOTION;

  // machine parameters
  b->machine = machine;
  b->acc = machine_A(b->machine);

  // fields to be calculated
  b->prof = malloc(sizeof(block_profile_t));
  if (!b->prof) {
    eprintf("Could not allocate memory for block profile\n");
    goto fail;
  }

  b->initial_point = point_new();
  b->target = point_new();
  b->delta = point_new();
  b->center = point_new();
  if (!b->center || !b->delta || !b->target) {
    eprintf("Could not allocate points in a new block\n");
    goto fail;
  }
  b->line = strdup(line);
  if (!b->line) {
    eprintf("Could not allocate memory for block line\n");
    goto fail;
  }
  return b;
fail:
  if (b)
    block_free(b);
  return NULL;
}

void block_free(block_t *b) {
  assert(b);
  if (b->line)
    free(b->line);
  if (b->prof)
    free(b->prof);
  if (b->target)
    point_free(b->target);
  if (b->delta)
    point_free(b->delta);
  if (b->center)
    point_free(b->center);
  if (b->initial_point)
    point_free(b->initial_point);
  free(b);
}

void block_print(block_t *b, FILE *out) {
  assert(b && out);
  char *start = NULL, *end = NULL;
  point_t *p0 = block_initial_point(b);
  point_inspect(p0, &start);
  point_inspect(b->target, &end);
  fprintf(out, "%03lu %s->%s F%7.1f S%7.1f T%2lu (G%02d) %f\n", b->n, start, end,
          b->feedrate, b->spindle, b->tool, b->type, b->trc);
  free(start);
  free(end);
}

// ACCESSORS (all getters) =====================================================

#define block_getter(typ, par, name)                                           \
  typ block_##name(block_t const *b) {                                         \
    assert(b);                                                                 \
    return b->par;                                                             \
  }

block_getter(data_t, length, length);
block_getter(data_t, dtheta, dtheta);
block_getter(data_t, prof->dt, dt);
block_getter(block_type_t, type, type);
block_getter(char *, line, line);
block_getter(size_t, n, n);
block_getter(data_t, r, r);
block_getter(point_t *, center, center);
block_getter(point_t *, target, target);
block_getter(block_t *, next, next);
block_getter(data_t,trc,trc);
block_getter(point_t *, initial_point, initial_point);

// METHODS =====================================================================

// we have a G-code line like "N10 G01 X0 Y100  z210.5 F1000 S5000"
int block_parse(block_t *b) {
  assert(b);
  point_t *p0 = NULL;
  int rv = 0;
  char *word, *line, *tofree;

  tofree = line = strdup(b->line);
  if (!line) {
    eprintf("Could not allocate memory for line string\n");
    return -1;
  }
  // tokenization
  while ((word = strsep(&line, " ")) != NULL) {
    // word[0] is the first character (the command)
    // word + 1 is the string beginning after the first character
    rv += block_set_fields(b, toupper(word[0]), word + 1);
  }
  free(tofree);

  // inherit coordinates from previous point
  p0 = start_point(b);
  point_modal(p0, b->initial_point);
  point_modal(p0, b->target);
  point_delta(p0, b->target, b->delta);
  b->length = point_dist(p0, b->target);

  #if TRC_FEATURE
  if(b->prev)
  if(block_trc(b->prev) != 0){
    point_t *p_new_target = point_new();
    
    switch (block_type(b->prev)) {
      case LINE:
        //set common parameters
        b->prev->acc = machine_A(b->machine);
        b->prev->arc_feedrate = b->feedrate;

        // line -> line / rapid
        if ((block_type(b) == LINE) || (block_type(b) == RAPID) || (block_type(b) == TRC_OFF)){

          // find intersection point and set as new target point of prevoius block

          //exception for G40 block
          if (block_type(b) == TRC_OFF) 
            b->trc = b->prev->trc;
            
          p_new_target = intersection_line_line (b->prev, b);
          point_clone(p_new_target, block_target(b->prev));

          // update initial point of prev block
          point_clone(start_point(b->prev), block_initial_point(b->prev));

          // update length
          p0 = start_point(b->prev);
          point_delta(p0, b->prev->target, b->prev->delta);
          b->prev->length = point_dist(p0, b->prev->target);

          //compute block after changes
          block_compute(b->prev);

          // restore trc value
          if (block_type(b) == TRC_OFF){
            b->trc = 0;
            point_clone(start_point(b),block_initial_point(b));
          }

          break;
        }

        // line -> arc 
        if ((block_type(b) == ARC_CCW) || (block_type(b) == ARC_CW)){
          if (block_arc(b)) {
            wprintf("Could not calculate arc coordinates\n");
            rv++;
            return rv;
          }
          
          // find intersection point and set it as new target point
          p_new_target = intersection_arc_line(b, b->prev, block_target(b->prev), false);
          point_clone(p_new_target, block_target(b->prev));

          // update initial point of prev block 
          point_clone(start_point(b->prev), block_initial_point(b->prev));

          block_compute(b->prev);
          break;
        }

      break;

      case ARC_CCW:
      case ARC_CW:
        //set common parameters
        // if (block_arc(b->prev)) {
        //     wprintf("Could not calculate arc coordinates\n");
        //     rv++;
        //     return rv;
        //   }
        b->prev->acc = machine_A(b->prev->machine) / 2.0;
        b->prev->arc_feedrate = MIN( b->prev->feedrate, pow(3.0 / 4.0 * pow(machine_A(b->prev->machine), 2) * pow(b->prev->r, 2), 0.25) * 60);

        // arc -> line / rapid
        if((block_type(b) == LINE) || (block_type(b) == RAPID) || (block_type(b) == TRC_OFF)){
          
          //exception for G40 block
          if (block_type(b) == TRC_OFF) 
            b->trc = b->prev->trc;

          // find intersection point and set it as new target point
          p_new_target = intersection_arc_line(b->prev, b, block_target(b->prev), true);
          point_clone(p_new_target, block_target(b->prev));

          // update initial point of prev block 
          point_clone(start_point(b->prev), block_initial_point(b->prev));

          if (block_arc(b->prev)) {
            wprintf("Could not calculate arc coordinates\n");
            rv++;
            return rv;
          }

          b->prev->arc_feedrate = MIN( b->prev->feedrate, pow(3.0 / 4.0 * pow(machine_A(b->prev->machine), 2) * pow(b->prev->r, 2), 0.25) * 60);
          
          // restore trc value
          if (block_type(b) == TRC_OFF){
            b->trc = 0;
            point_clone(start_point(b),block_initial_point(b));
          }

          return rv;
        }
      
        if (block_type(b) == TRC_OFF)
          point_clone(start_point(b),block_initial_point(b));

      break;

      case RAPID:
      case TRC_LEFT:
      case TRC_RIGHT:
        if ((block_type(b) == LINE) || (block_type(b) == TRC_OFF)){
          p_new_target = intersection_line_line(b->prev, b);

          point_clone(p_new_target, block_target(b->prev));

          // update initial point of prev block
          point_clone(start_point(b->prev), block_initial_point(b->prev));

          if (block_type(b) == TRC_OFF)
            point_clone(start_point(b),block_initial_point(b));

          return rv;
        }

        if ((block_type(b) == ARC_CCW) || (block_type(b) == ARC_CW)){
          if (block_arc(b)) {
            wprintf("Could not calculate arc coordinates\n");
            rv++;
            return rv;
          }
          
          p_new_target = intersection_arc_line(b, b->prev, block_target(b->prev), false);

          // update final point of rapid block
          point_set_x(block_target(b->prev), point_x(p_new_target));
          point_set_y(block_target(b->prev), point_y(p_new_target));

          // update initial point of prev block
          point_set_x (block_initial_point(b->prev), point_x(start_point(b->prev)));
          point_set_y (block_initial_point(b->prev), point_y(start_point(b->prev)));
          return rv;
        }

      break;

      default:
      break;
    }
    return rv;
  }

  // if we have a G40 block we need to fix the initial point with the previous target point

  //point_set_xyz(block_initial_point(b), point_x(start_point(b)), point_y(start_point(b)), point_z(start_point(b)));

  #endif

  // no trc - original block_parse() function
  switch (b->type) {
  case LINE:
    b->acc = machine_A(b->machine);
    b->arc_feedrate = b->feedrate;
    block_compute(b);
    break;

  case ARC_CW:
  case ARC_CCW:
     if (block_arc(b)) {
      wprintf("Could not calculate arc coordinates\n");
      rv++;
      break;
    }
    b->acc = machine_A(b->machine) / 2.0;
    b->arc_feedrate = MIN(b->feedrate, pow(3.0 / 4.0 * pow(machine_A(b->machine), 2) * pow(b->r, 2), 0.25) * 60);
    block_compute(b);
    break;
  default:
    break;
  }
  return rv;
}

data_t block_lambda(block_t *b, data_t t, data_t *v) {
  assert(b);
  data_t r;
  data_t dt_1 = b->prof->dt_1;
  data_t dt_2 = b->prof->dt_2;
  data_t dt_m = b->prof->dt_m;
  data_t a = b->prof->a;
  data_t d = b->prof->d;
  data_t f = b->prof->f;

  if (t < 0) {
    r = 0.0;
    *v = 0.0;
  } else if (t < dt_1) { // acceleration
    r = a * pow(t, 2) / 2.0;
    *v = a * t;
  } else if (t < dt_1 + dt_m) { // maintenance
    r = f * (dt_1 / 2.0 + (t - dt_1));
    *v = f;
  } else if (t < dt_1 + dt_m + dt_2) { // deceleration
    data_t t_2 = dt_1 + dt_m;
    r = f * dt_1 / 2.0 + f * (dt_m + t - t_2) +
        d / 2.0 * (pow(t, 2) + pow(t_2, 2)) - d * t * t_2;
    *v = f + d * (t - t_2);
  } else {
    r = b->prof->l;
    *v = 0.0;
  }

  r /= b->prof->l;
  *v *= 60; // convert to mm/min
  return r;
}

point_t *block_interpolate(block_t *b, data_t lambda) {
  assert(b);
  point_t *result = machine_setpoint(b->machine);
  point_t *p0 = block_initial_point(b);

  // Parametric equations of segment:
  // x(t) = x(0) + d_x * lambda
  // y(t) = y(0) + d_y * lambda
  if (b->type == LINE) {
    point_set_x(result, point_x(p0) + point_x(b->delta) * lambda);
    point_set_y(result, point_y(p0) + point_y(b->delta) * lambda);
  }
  // paremetric equations of arc:
  // x(t) = x_c + R cos(theta_0 + dtheta * lambda)
  // y(t) = y_c + R sin(theta_0 + dtheta * lambda)
  else if (b->type == ARC_CW || b->type == ARC_CCW) {
    point_set_x(result, point_x(b->center) +
                            b->r * cos(b->theta0 + b->dtheta * lambda));
    point_set_y(result, point_y(b->center) +
                            b->r * sin(b->theta0 + b->dtheta * lambda));
  } else {
    wprintf("Unexpected block type\n");
    return NULL;
  }
  // Z is always linearly intepolated (arc becomes spiral)
  point_set_z(result, point_z(p0) + point_z(b->delta) * lambda);
  return result;
}

// Static functions
//   ____  _        _   _         __                  _   _
//  / ___|| |_ __ _| |_(_) ___   / _|_   _ _ __   ___| |_(_) ___  _ __  ___
//  \___ \| __/ _` | __| |/ __| | |_| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
//   ___) | || (_| | |_| | (__  |  _| |_| | | | | (__| |_| | (_) | | | \__ \
//  |____/ \__\__,_|\__|_|\___| |_|  \__,_|_| |_|\___|\__|_|\___/|_| |_|___/

// Return a reliable previous point, i.e. machine zero if this is the first
// block
static point_t *start_point(block_t *b) {
  assert(b);
  return b->prev ? b->prev->target : machine_zero(b->machine);
}

static int block_set_fields(block_t *b, char cmd, char *arg) {
  assert(b && arg);
  switch (cmd) {
  case 'N':
    b->n = atol(arg);
    break;
  case 'G':
    # if TRC_FEATURE
    block_trc_evaluation(b, arg);
    #endif
    b->type = atoi(arg);
    break;
  case 'X':
    point_set_x(b->target, atof(arg));
    break;
  case 'Y':
    point_set_y(b->target, atof(arg));
    break;
  case 'Z':
    point_set_z(b->target, atof(arg));
    break;
  case 'I':
    b->i = atof(arg);
    break;
  case 'J':
    b->j = atof(arg);
    break;
  case 'R':
    b->r = atof(arg);
    break;
  case 'F':
    if (strcmp(arg, "MAX") == 0) {
      b->feedrate = machine_fmax(b->machine);
    } else {
      b->feedrate = MIN(atof(arg), machine_fmax(b->machine));
    }
    break;
  case 'S':
    b->spindle = atof(arg);
    break;
  case 'T':
    b->tool = atol(arg);
    break;

  default:
    wprintf("Unsupported command %c\n", cmd);
    return 1;
    break;
  }
  // Both R and IJ are specified
  if (b->r && (b->i || b->j)) {
    wprintf("Cannot mix R and I,J\n");
    return 1;
  }
  return 0;
}

static data_t quantize(data_t t, data_t tq, data_t *dq) {
  data_t q;
  q = ((size_t)(t / tq) + 1) * tq;
  *dq = q - t;
  return q;
}

static void block_compute(block_t *b) {
  assert(b);
  data_t A, a, d;
  data_t dt, dt_1, dt_2, dt_m, dq;
  data_t f_m, l;

  A = b->acc;
  f_m = b->arc_feedrate / 60.0;
  l = b->length;
  dt_1 = f_m / A;
  dt_2 = dt_1;
  dt_m = l / f_m - (dt_1 + dt_2) / 2.0;
  if (dt_m > 0) { // trapezoidal profile
    dt = quantize(dt_1 + dt_m + dt_2, machine_tq(b->machine), &dq);
    dt_m += dq;
    f_m = (2 * l) / (dt_1 + dt_2 + 2 * dt_m);
  } else { // triangular profile (short block)
    dt_1 = dt_2 = sqrt(l / A);
    dt = quantize(dt_1 + dt_2, machine_tq(b->machine), &dq);
    dt_m = 0;
    dt_2 += dq;
    f_m = 2 * l / (dt_1 + dt_2);
  }
  a = f_m / dt_1;
  d = -(f_m / dt_2);
  // copy back values into block object
  b->prof->dt_1 = dt_1;
  b->prof->dt_2 = dt_2;
  b->prof->dt_m = dt_m;
  b->prof->a = a;
  b->prof->d = d;
  b->prof->f = f_m;
  b->prof->dt = dt;
  b->prof->l = l;
}

// Calculate the arc coordinates
// see slides pages 107-109
static int block_arc(block_t *b) {
  data_t x0, y0, z0, xc, yc, xf, yf, zf, r;
  point_t *p0 = start_point(b);
  x0 = point_x(p0);
  y0 = point_y(p0);
  z0 = point_z(p0);
  xf = point_x(b->target);
  yf = point_y(b->target);
  zf = point_z(b->target);

  if (b->r) { // if the radius is given
    data_t dx = point_x(b->delta);
    data_t dy = point_y(b->delta);
    r = b->r;
    data_t dxy2 = pow(dx, 2) + pow(dy, 2);
    data_t sq = sqrt(-pow(dy, 2) * dxy2 * (dxy2 - 4 * r * r));
    // signs table
    // sign(r) | CW(-1) | CCW(+1)
    // --------------------------
    //      -1 |     +  |    -
    //      +1 |     -  |    +
    int s = (r > 0) - (r < 0);
    s *= (b->type == ARC_CCW ? 1 : -1);
    xc = x0 + (dx - s * sq / dxy2) / 2.0;
    yc = y0 + dy / 2.0 + s * (dx * sq) / (2 * dy * dxy2);
  }
  else { // if I,J are given
    data_t r2;
    r = hypot(b->i, b->j);
    xc = x0 + b->i;
    yc = y0 + b->j;
    r2 = hypot(xf - xc, yf - yc);
    if (fabs(r - r2) > machine_error(b->machine)) {
      fprintf(stderr, "Arc endpoints mismatch error (%f)\n", r - r2);
      return 1;
    }
    b->r = r;
  }
  point_set_x(b->center, xc);
  point_set_y(b->center, yc);
  b->theta0 = atan2(y0 - yc, x0 - xc);
  b->dtheta = atan2(yf - yc, xf - xc) - b->theta0;
  // we need the net angle so we take the 2PI complement if negative
  if (b->dtheta <0) 
    b->dtheta = 2 * M_PI + b->dtheta;
  // if CW, take the negative complement
  if (b->type == ARC_CW)
    b->dtheta = -(2 * M_PI - b->dtheta);
  //
  b->length = hypot(zf - z0, b->dtheta * b->r);
  // from now on, it's safer to drop the sign of radius angle
  b->r = fabs(b->r);
  return 0;
}

#if TRC_FEATURE

/**
 * Set value of trc and return the block type
 * 
 * @param b block to be analyzed
 * @param arg char with GCODE of trc
 * 
 * @return the block type
*/
static void block_trc_evaluation(block_t *b, char *arg){
  switch (atoi(arg)){
    case 40: b->trc = 0; break;
    case 41: b->trc = 1; break;
    case 42: b->trc = -1; break;
    default: break;
  }
  return;
}

/**
 * Returns the equation of the block when it is a line:
 *  y = a*x + b
 *
 * @param a is the coefficient of x
 * @param b is the free term
 * @param p_init is the initial point of the line
 * @param p_final is the final point of the line
 * 
 * @return if the equation is the special case x = a, if TRUE, the equation is x = a
 * 
 * @if TRUE, the equation is x = a
 * 
 */
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

  *a = temp_a;
  *b = temp_b;
  return vertical;
}

/**
 * 
 * Evaluate the equation of the line sign through two points
 * 
 * @param from is the starting point
 * @param to is the final point
 * @param trc is the tool radius compensation of the block 
 * 
 * @return if we have to add or subtract the quatity for tool radius compensation (+-1)
 * 
*/
static int block_eq_sign(point_t *from, point_t *to, data_t const trc){

  // if trc = 1  => right 
  // if trc = 0  => no
  // if trc = -1 => left
  point_t *p_dist = point_new();
  point_delta(from, to, p_dist);

  // z must be ignored

  point_set_z(p_dist, 0);

  if((point_x(p_dist) > 0) && (point_y(p_dist) >= 0)) return -trc;
  if((point_x(p_dist) <= 0) && (point_y(p_dist) >= 0)) return trc;
  if((point_x(p_dist) >= 0) && (point_y(p_dist) <  0)) return -trc;
  if((point_x(p_dist) < 0) && (point_y(p_dist) < 0)) return trc;

  return 0;

}

/**
 * Calculate the intersection point between an arc block and a line block (with trc evaluation).
 * 
 * @param b_arc is the arc block
 * @param b_line is the line block
 * @param target is the original intersection point
 * @param change_radius says whether or not the arc block's radius can be changed
 * 
 * @return The intersection point of the two blocks.
 * 
*/
static point_t *intersection_arc_line(block_t *b_arc, block_t *b_line, point_t const *target, bool change_radius){
  // tool radius
  data_t tool_radius = machine_tool_radius (b_arc->machine, b_arc->tool);

  //line
  bool vertical = false;
  data_t a, b;

  //arc
  point_t *center_point = block_center(b_arc);
  point_t *p1 = point_new();
  point_t *p2 = point_new();
  data_t x1, x2, y1, y2, temp_radius;

  //line equation
  vertical = block_equation(&a, &b, block_initial_point(b_line), block_target(b_line));
  int sign_trc = block_eq_sign(block_initial_point(b_line), block_target(b_line), b_line->trc);
  b += sign_trc * tool_radius * sqrt(a * a + 1.0);

  //evaluate arc block
  b_arc->acc = machine_A(b_arc->machine) / 2.0;
  b_arc->arc_feedrate = MIN( b_arc->feedrate, pow(3.0 / 4.0 * pow(machine_A(b_arc->machine), 2) * pow(b_arc->r, 2), 0.25) * 60);

  // trc reduces or increses the radius of the arc
        
  // sign(r+-r_tool) |    CW       |   CCW
  // -------------------------------------------
  // RIGHT TRC (+1)  | r - r_tool  | r - r_tool
  // LEFT  TRC (-1)  | r + r_tool  | r + r_tool  
  
  // get new radius and set it
  temp_radius = (b_arc->type == ARC_CCW) ? (b_arc->r + b_arc->trc * tool_radius) : (b_arc->r - b_arc->trc * tool_radius);

  if(change_radius)
    b_arc->r = temp_radius;

  // intersection point evaluation
  if(vertical){
    x1 = point_x(block_target(b_line)) + sign_trc * tool_radius;
          
    point_set_x(p1, x1);
    point_set_x(p2, x1);

    // y = yc +- sqrt(r^2 - (x-xc)^2)
    y1 = point_y(center_point) + sqrt(pow(temp_radius,2) - pow(x1 - point_x(center_point),2));
    y2 = point_y(center_point) - sqrt(pow(temp_radius,2) - pow(x1 - point_x(center_point),2));

    point_set_y(p1, y1);
    point_set_y(p2, y2);
          
    //set closest point as new target 
    return (point_dist(target, p1) < point_dist(target, p2)) ? p1 : p2;

  }

  // generic case (not vertical)

  /* mathematical computation
  After some iterations the equation of the intersection is the following:
  (a2^2 + 1)x^2 + 2(a2 * b2 - Xc - a2 * Yc)x + Xc^2 + b2^2 + Yc^2 -2 * b2 * Yc - r^2 = 0
  
  With the formula x = (-b +- sqrt(b^2 - 4 * a * c))/ (2 * a) there are two possible solutions
  */

  // A = (a2^2 + 1)
  data_t A = pow(a, 2) + 1;

  // B = 2(a2 * b2 - Xc - a2 * Yc)
  data_t B = 2 * (a * (b - point_y(center_point)) - point_x(center_point));

  // C = +Xc^2 + b2^2 + Yc^2 -2 * b2 * Yc - r^2
  data_t C = + pow(point_x(center_point), 2) 
             + pow(b, 2) + pow(point_y(center_point), 2) 
             - 2 * b * point_y(center_point) 
             - pow(temp_radius, 2);

  x1 = (-B + sqrt(pow(B,2) - 4 * A * C)) / 2 * A;
  x2 = (-B - sqrt(pow(B,2) - 4 * A * C)) / 2 * A;

  point_set_x(p1, x1);
  point_set_x(p2, x2);

  // use eq of line to find y
  y1 = a * x1 + b;
  y2 = a * x2 + b;

  point_set_y(p1, y1);
  point_set_y(p2, y2);

  //return closest point as new target 
  return (point_dist(target, p1) < point_dist(target, p2)) ? p1 : p2;

}

/**
 * Calculate the intersection point between two line blocks (with trc evaluation).
 * 
 * @param b_line1 is the first line block
 * @param b_line2 is the second line block
 * 
 * @return The intersection point of the two blocks.
 * 
*/
static point_t *intersection_line_line(block_t *b_line1, block_t *b_line2){
  // tool radius
  data_t tool_radius = machine_tool_radius (b_line1->machine, b_line1->tool);

  bool vertical1 = false;
  bool vertical2 = false;
  data_t a1, a2, b1, b2, x, y;
  point_t *p_new_target = point_new();

  vertical1 = block_equation(&a1, &b1, block_initial_point(b_line1), block_target(b_line1));
  vertical2 = block_equation(&a2, &b2, block_initial_point(b_line2), block_target(b_line2));
  int sign_trc = block_eq_sign(block_initial_point(b_line1), block_target(b_line1), block_trc(b_line1));
  int sign_trc1 = block_eq_sign(block_initial_point(b_line2),block_target(b_line2), block_trc(b_line2));

  //does not affect the vertical lines
  b1 += sign_trc * tool_radius * sqrt(a1 * a1 + 1.0);
  b2 += sign_trc1 * tool_radius * sqrt(a2 * a2 + 1.0);

  // vertical lines
  if(!vertical1 && vertical2){
    x = point_x(block_target(b_line2)) + sign_trc1 * tool_radius;
    // y = m * x + q
    y = a1 * x + b1;
    point_set_xyz(p_new_target, x, y, point_z(block_target(b_line1)));
    return p_new_target;
  }

  if(vertical1 && !vertical2){
    x = point_x(block_target(b_line1)) + sign_trc1 * tool_radius;
    // y = m * x + q
    y = a2 * x + b2;
    point_set_xyz(p_new_target, x, y, point_z(block_target(b_line1)));
    return p_new_target;
  }

  if(vertical1 && vertical2){
    x = point_x(block_target(b_line1)) + sign_trc * tool_radius;
    y = point_y(block_target(b_line1));
    point_set_xyz(p_new_target, x, y, point_z(block_target(b_line1)));
    return p_new_target;
  }

  // not vertical lines
  // collinear case - horizontal case
  if((a1 == 0) && (a2 == 0)){
    x = point_x(block_target(b_line1));
    y = point_y(block_target(b_line1)) + sign_trc * tool_radius;
    point_set_xyz(p_new_target, x, y, point_z(block_target(b_line1)));
    return p_new_target;
  } 

  // generic collinear case
  if (a1 == a2){
    x = sign_trc * a1 / fabs(a1) * sqrt((a1 * a1) / (a1 * a1 + 1));
    y = -x / a1;
    point_set_xyz(p_new_target, x, y, point_z(block_target(b_line1)));
    return p_new_target;
  }
  
  // generic case
  x = (b2 - b1) / (a1 - a2);
  y = (b1 * a2 - b2 * a1) / (a2 - a1);
  point_set_xyz(p_new_target, x, y, point_z(block_target(b_line1)));
  return p_new_target;
}

#endif

// MAIN
//   _____         _                     _
//  |_   _|__  ___| |_   _ __ ___   __ _(_)_ __
//    | |/ _ \/ __| __| | '_ ` _ \ / _` | | '_ \
//    | |  __/\__ \ |_  | | | | | | (_| | | | | |
//    |_|\___||___/\__| |_| |_| |_|\__,_|_|_| |_|

#ifdef BLOCK_MAIN


//test 1
#if 0
int main(int argc, char const *argv[]) {
  machine_t *m = machine_new(argv[1]);
  block_t *b1 = NULL, *b2 = NULL, *b3 = NULL, *b4 = NULL;
  if (!m) {
    eprintf("Error creating machine\n");
    exit(EXIT_FAILURE);
  }

  b1 = block_new("N10 G41 x-10 y0", NULL, m);
  block_parse(b1);
  b2 = block_new("N20 G01 y100 S2000", b1, m);
  block_parse(b2);
  b3 = block_new("N30 G01 Y200", b2, m);
  block_parse(b3);
  b4 = block_new("N40 G01 x0 y0 z0", b3, m);
  block_parse(b4);

  block_print(b1, stderr);
  block_print(b2, stderr);
  block_print(b3, stderr);
  block_print(b4, stderr);

  wprintf("Intepolation of block 20 (duration: %f)\n", block_dt(b2));
  {
    data_t t = 0, tq = machine_tq(m), dt = block_dt(b2);
    data_t lambda = 0, v = 0;
    printf("t lambda v x y z\n");
    for (t = 0; t - dt <= tq/10.0; t += tq) {
      lambda = block_lambda(b2, t, &v);
      block_interpolate(b2, lambda);
      printf("%f %f %f %.3f %.3f %.3f\n", t, lambda, v, 
        point_x(machine_setpoint(m)),
        point_y(machine_setpoint(m)),
        point_z(machine_setpoint(m)));
    }
  }

  block_free(b1);
  block_free(b2);
  block_free(b3);
  block_free(b4);
  machine_free(m);
  return 0;
}

#endif

//test collinear blocks
#if 0
int main(int argc, char const *argv[]) {
  machine_t *m = machine_new(argv[1]);
  block_t *b1 = NULL, *b2 = NULL, *b3 = NULL, *b4 = NULL, *b5 =NULL;
  if (!m) {
    eprintf("Error creating machine\n");
    exit(EXIT_FAILURE);
  }

  b1 = block_new("N10 G42 x5 Z20 T1", NULL, m);
  block_parse(b1);
  b2 = block_new("N20 G01 x10 z0 f200", b1, m);
  block_parse(b2);
  b3 = block_new("N30 g01 y20 f2000 s5000 t1", b2, m);
  block_parse(b3);
  b4 = block_new("N40 g01 x40", b3, m);
  block_parse(b4);
  b5 = block_new("N50 g40 x0 y0 z200", b4, m);
  block_parse(b5);

  block_print(b1, stderr);
  block_print(b2, stderr);
  block_print(b3, stderr);
  block_print(b4, stderr);

  wprintf("Intepolation of block 20 (duration: %f)\n", block_dt(b2));
  {
    data_t t = 0, tq = machine_tq(m), dt = block_dt(b2);
    data_t lambda = 0, v = 0;
    printf("t lambda v x y z\n");
    for (t = 0; t - dt <= tq/10.0; t += tq) {
      lambda = block_lambda(b2, t, &v);
      block_interpolate(b2, lambda);
      printf("%f %f %f %.3f %.3f %.3f\n", t, lambda, v, 
        point_x(machine_setpoint(m)),
        point_y(machine_setpoint(m)),
        point_z(machine_setpoint(m)));
    }
  }

  block_free(b1);
  block_free(b2);
  block_free(b3);
  block_free(b4);
  machine_free(m);
  return 0;
}
#endif

//test vertical blocks
#if 0
int main(int argc, char const *argv[]) {
  machine_t *m = machine_new(argv[1]);
  block_t *b1 = NULL, *b2 = NULL, *b3 = NULL, *b4 = NULL, *b5 =NULL, *b6 = NULL;
  if (!m) {
    eprintf("Error creating machine\n");
    exit(EXIT_FAILURE);
  }

  b1 = block_new("N10 G42 x5 Z20 T1", NULL, m);
  block_parse(b1);
  b2 = block_new("N20 G01 x10 z0 f200", b1, m);
  block_parse(b2);
  b3 = block_new("N30 g01 x20 f2000 s5000", b2, m);
  block_parse(b3);
  b4 = block_new("N40 g01 x40", b3, m);
  block_parse(b4);
  b5 = block_new("N50 g01 x60", b4, m);
  block_parse(b5);
  b6 = block_new("N60 g40 x70", b5, m);
  block_parse(b6);

  block_print(b1, stderr);
  block_print(b2, stderr);
  block_print(b3, stderr);
  block_print(b4, stderr);
  block_print(b5, stderr);
  block_print(b6, stderr);

  wprintf("Intepolation of block 20 (duration: %f)\n", block_dt(b2));
  {
    data_t t = 0, tq = machine_tq(m), dt = block_dt(b2);
    data_t lambda = 0, v = 0;
    printf("t lambda v x y z\n");
    for (t = 0; t - dt <= tq/10.0; t += tq) {
      lambda = block_lambda(b2, t, &v);
      block_interpolate(b2, lambda);
      printf("%f %f %f %.3f %.3f %.3f\n", t, lambda, v, 
        point_x(machine_setpoint(m)),
        point_y(machine_setpoint(m)),
        point_z(machine_setpoint(m)));
    }
  }

  block_free(b1);
  block_free(b2);
  block_free(b3);
  block_free(b4);
  machine_free(m);
  return 0;
}
#endif

// original main
#if 0
int main(int argc, char const *argv[]) {
  machine_t *m = machine_new(argv[1]);
  block_t *b1 = NULL, *b2 = NULL, *b3 = NULL, *b4 = NULL;
  if (!m) {
    eprintf("Error creating machine\n");
    exit(EXIT_FAILURE);
  }

  b1 = block_new("N10 G00 X90 Y90 Z100 T3 F1000", NULL, m);
  block_parse(b1);
  b2 = block_new("N20 G01 y100 S2000", b1, m);
  block_parse(b2);
  b3 = block_new("N30 G01 Y200", b2, m);
  block_parse(b3);
  b4 = block_new("N40 G01 x0 y0 z0", b3, m);
  block_parse(b4);

  block_print(b1, stderr);
  block_print(b2, stderr);
  block_print(b3, stderr);
  block_print(b4, stderr);

  wprintf("Intepolation of block 20 (duration: %f)\n", block_dt(b2));
  {
    data_t t = 0, tq = machine_tq(m), dt = block_dt(b2);
    data_t lambda = 0, v = 0;
    printf("t lambda v x y z\n");
    for (t = 0; t - dt <= tq/10.0; t += tq) {
      lambda = block_lambda(b2, t, &v);
      block_interpolate(b2, lambda);
      printf("%f %f %f %.3f %.3f %.3f\n", t, lambda, v, 
        point_x(machine_setpoint(m)),
        point_y(machine_setpoint(m)),
        point_z(machine_setpoint(m)));
    }
  }

  block_free(b1);
  block_free(b2);
  block_free(b3);
  block_free(b4);
  machine_free(m);
  return 0;
}
#endif

// test arc
#if 1
int main(int argc, char const *argv[]) {
  machine_t *m = machine_new(argv[1]);
  block_t *b1 = NULL, *b2 = NULL, *b3 = NULL, *b4 = NULL, *b5 =NULL, *b6 = NULL;
  if (!m) {
    eprintf("Error creating machine\n");
    exit(EXIT_FAILURE);
  }

  b1 = block_new("N10 G41 x5 Z0 T1", NULL, m);
  block_parse(b1);
  b2 = block_new("N20 G01 x10 f200", b1, m);
  block_parse(b2);
  b3 = block_new("N30 g03 x20 y10 r10 f2000 s5000", b2, m);
  block_parse(b3);
  b4 = block_new("N40 g01 y40", b3, m);
  block_parse(b4);
  b5 = block_new("N50 g01 x0", b4, m);
  block_parse(b5);
  b6 = block_new("N60 g40 y0", b5, m);
  block_parse(b6);

  block_print(b1, stderr);
  block_print(b2, stderr);
  block_print(b3, stderr);
  block_print(b4, stderr);
  block_print(b5, stderr);
  block_print(b6, stderr);

  wprintf("Intepolation of block 20 (duration: %f)\n", block_dt(b2));
  {
    data_t t = 0, tq = machine_tq(m), dt = block_dt(b2);
    data_t lambda = 0, v = 0;
    printf("t lambda v x y z\n");
    for (t = 0; t - dt <= tq/10.0; t += tq) {
      lambda = block_lambda(b2, t, &v);
      block_interpolate(b2, lambda);
      printf("%f %f %f %.3f %.3f %.3f\n", t, lambda, v, 
        point_x(machine_setpoint(m)),
        point_y(machine_setpoint(m)),
        point_z(machine_setpoint(m)));
    }
  }

  block_free(b1);
  block_free(b2);
  block_free(b3);
  block_free(b4);
  machine_free(m);
  return 0;
}
#endif


#endif